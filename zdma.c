/*
 * Xilinx AXI DMA Engine support
 *
 * Copyright (C) 2013 Yohei Matsumoto. All rights reserved.
 *
 * Based on the Xilinx AXI DMA driver.
 *
 * Description:
 *  . Axi DMA engine, it does transfers between memory and device. It can be
 *    configured to have one channel or two channels. If configured as two
 *    channels, one is to transmit to a device and another is to receive from
 *    a device.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/amba/xilinx_dma.h>

/* Hw specific definitions */
#define XDMA_MAX_CHANS_PER_DEVICE	0x2
#define XDMA_MAX_TRANS_LEN	0x7FFFFF

/* General register bits definitions */
#define XDMA_CR_RESET_MASK	0x00000004
/* Reset DMA engine */
#define XDMA_CR_RUNSTOP_MASK	0x00000001
/* Start/stop DMA engine */

#define XDMA_SR_HALTED_MASK	0x00000001
/* DMA channel halted */
#define XDMA_SR_IDLE_MASK	0x00000002
/* DMA channel idle */

#define XDMA_SR_ERR_INTERNAL_MASK 0x00000010
/* Datamover internal err */
#define XDMA_SR_ERR_SLAVE_MASK	0x00000020
/* Datamover slave err */
#define XDMA_SR_ERR_DECODE_MASK	0x00000040
/* Datamover decode err */
#define XDMA_SR_ERR_SG_INT_MASK	0x00000100
/* SG internal err */
#define XDMA_SR_ERR_SG_SLV_MASK	0x00000200
/* SG slave err */
#define XDMA_SR_ERR_SG_DEC_MASK	0x00000400
/* SG decode err */
#define XDMA_SR_ERR_ALL_MASK	0x00000770
/* All errors */

#define XDMA_XR_IRQ_IOC_MASK	0x00001000
/* Completion interrupt */
#define XDMA_XR_IRQ_DELAY_MASK	0x00002000
/* Delay interrupt */
#define XDMA_XR_IRQ_ERROR_MASK	0x00004000
/* Error interrupt */
#define XDMA_XR_IRQ_ALL_MASK	0x00007000
/* All interrupts */

#define XDMA_XR_DELAY_MASK	0xFF000000
/* Delay timeout counter */
#define XDMA_XR_COALESCE_MASK	0x00FF0000
/* Coalesce counter */

#define XDMA_IRQ_SHIFT		12
#define XDMA_DELAY_SHIFT      	24
#define XDMA_COALESCE_SHIFT	16

#define XDMA_DELAY_MAX		0xFF
/* Maximum delay counter value */
#define XDMA_COALESCE_MAX	0xFF
/* Maximum coalescing counter value */

#define XDMA_RX_CHANNEL_OFFSET	0x30

/* BD definitions for AXI Dma */
#define XDMA_BD_STS_COMPL_MASK	0x80000000
#define XDMA_BD_STS_ERR_MASK	0x70000000
#define XDMA_BD_STS_ALL_MASK	0xF0000000

/* Axi DMA BD special bits definitions */
#define XDMA_BD_SOP	0x08000000	/* Start of packet bit */
#define XDMA_BD_EOP	0x04000000	/* End of packet bit */

/* Feature encodings */
#define XDMA_FTR_DATA_WIDTH_MASK	0x000000FF
/* Data width mask, 1024 */
#define XDMA_FTR_HAS_SG		0x00000100
/* Has SG */
#define XDMA_FTR_HAS_SG_SHIFT	8
/* Has SG shift */
#define XDMA_FTR_STSCNTRL_STRM	0x00010000
/* Optional feature for dma */

/* Delay loop counter to prevent hardware failure */
#define XDMA_RESET_LOOP		1000000
#define XDMA_HALT_LOOP		1000000

/* Device Id in the private structure */
#define XDMA_DEVICE_ID_SHIFT	28

/* IO accessors */
#define DMA_OUT(addr, val)	(iowrite32(val, addr))
#define DMA_IN(addr)		(ioread32(addr))

#ifdef CONFIG_XDMATEST
#define TEST_DMA_WITH_LOOPBACK
#endif

/* Hardware descriptor */
struct xdma_desc_hw {
  u32 next_desc;	/* 0x00 */
  u32 pad1;	/* 0x04 */
  u32 buf_addr;	/* 0x08 */
  u32 pad2;	/* 0x0C */
  u32 pad3;	/* 0x10 */
  u32 pad4;	/* 0x14 */
  u32 control;	/* 0x18 */
  u32 status;	/* 0x1C */
  u32 app_0;	/* 0x20 */
  u32 app_1;	/* 0x24 */
  u32 app_2;	/* 0x28 */
  u32 app_3;	/* 0x2C */
  u32 app_4;	/* 0x30 */
} __aligned(64);

/* Software descriptor */
struct xdma_desc_sw {
  struct xdma_desc_hw hw;// 52byte?
  struct list_head node; // 8byte?
  struct list_head tx_list; // 8byte?
  struct dma_async_tx_descriptor async_tx; // 28byte?
} __aligned(64);

/* AXI DMA Registers Structure */
struct xdma_regs {
  u32 cr;		/* 0x00 Control Register */
  u32 sr;		/* 0x04 Status Register */
  u32 cdr;	/* 0x08 Current Descriptor Register */
  u32 pad1;
  u32 tdr;	/* 0x10 Tail Descriptor Register */
  u32 pad2;
  u32 src;	/* 0x18 Source Address Register (sg = 0) */
  u32 pad3;
  u32 dst;	/* 0x20 Destination Address Register (sg = 0) */
  u32 pad4;
  u32 btt_ref;	/* 0x28 Bytes To Transfer (sg = 0) */
};

/* Per DMA specific operations should be embedded in the channel structure */
struct xdma_chan {
  struct xdma_regs __iomem *regs;	/* Control status registers */
  dma_cookie_t completed_cookie;	/* The maximum cookie completed */
  dma_cookie_t cookie;		/* The current cookie */
  spinlock_t lock;		/* Descriptor operation lock */
  bool sg_waiting;		/* Scatter gather transfer waiting */
  struct list_head active_list;	/* Active descriptors */
  struct list_head pending_list;	/* Descriptors waiting */
  struct dma_chan common;		/* DMA common channel */
  struct dma_pool *desc_pool;	/* Descriptors pool */
  struct device *dev;		/* The dma device */
  int irq;			/* Channel IRQ */
  int id;				/* Channel ID */
  enum dma_transfer_direction direction;
  /* Transfer direction */
  int max_len;			/* Maximum data len per transfer */
  int is_lite;			/* Whether is light build */
  int has_SG;			/* Support scatter transfers */
  int has_DRE;			/* Support unaligned transfers */
  int err;			/* Channel has errors */
  struct tasklet_struct tasklet;	/* Cleanup work after irq */
  u32 feature;			/* IP feature */
  u32 private;			/* Match info for channel request */
  void (*start_transfer)(struct xdma_chan *chan);
  struct xilinx_dma_config config;
  /* Device configuration info */
};

/* DMA Device Structure */
struct xdma_device {
  void __iomem *regs;
  struct device *dev;
  struct dma_device common;
  struct xdma_chan **chan;
  u32 feature;
  int irq;
  /* modified by yohei matsumoto*/
  int has_mm2s;
  int num_mm2s_chans;
  int has_s2mm;
  int num_s2mm_chans;
};

#define to_xilinx_chan(chan)			\
  container_of(chan, struct xdma_chan, common)

/* Required functions */

static int xdma_alloc_chan_resources(struct dma_chan *dchan)
{
  struct xdma_chan *chan = to_xilinx_chan(dchan);

  /* Has this channel already been allocated? */
  if (chan->desc_pool)
    return 1;

  /*
   * We need the descriptor to be aligned to 64bytes
   * for meeting Xilinx DMA specification requirement.
   */
  chan->desc_pool = dma_pool_create("xdma_desc_pool",
				    chan->dev,
				    sizeof(struct xdma_desc_sw),
				    __alignof__(struct xdma_desc_sw), 0);
  if (!chan->desc_pool) {
    dev_err(chan->dev,
	    "unable to allocate channel %d descriptor pool\n",
	    chan->id);
    return -ENOMEM;
  }

  chan->completed_cookie = 1;
  chan->cookie = 1;

  /* There is at least one descriptor free to be allocated */
  return 1;
}

static void xdma_free_desc_list(struct xdma_chan *chan,
				struct list_head *list)
{
  struct xdma_desc_sw *desc, *_desc;

  list_for_each_entry_safe(desc, _desc, list, node) {
    list_del(&desc->node);
    dma_pool_free(chan->desc_pool, desc, desc->async_tx.phys);
  }
}

static void xdma_free_desc_list_reverse(struct xdma_chan *chan,
					struct list_head *list)
{
  struct xdma_desc_sw *desc, *_desc;

  list_for_each_entry_safe_reverse(desc, _desc, list, node) {
    list_del(&desc->node);
    dma_pool_free(chan->desc_pool, desc, desc->async_tx.phys);
  }
}

static void xdma_free_chan_resources(struct dma_chan *dchan)
{
  struct xdma_chan *chan = to_xilinx_chan(dchan);
  unsigned long flags;

  dev_dbg(chan->dev, "Free all channel resources.\n");
  spin_lock_irqsave(&chan->lock, flags);
  xdma_free_desc_list(chan, &chan->active_list);
  xdma_free_desc_list(chan, &chan->pending_list);
  spin_unlock_irqrestore(&chan->lock, flags);

  dma_pool_destroy(chan->desc_pool);
  chan->desc_pool = NULL;
}

static enum dma_status xdma_desc_status(struct xdma_chan *chan,
					struct xdma_desc_sw *desc)
{
  return dma_async_is_complete(desc->async_tx.cookie,
			       chan->completed_cookie,
			       chan->cookie);
}

static void xilinx_chan_desc_cleanup(struct xdma_chan *chan)
{
  struct xdma_desc_sw *desc, *_desc;
  unsigned long flags;

  spin_lock_irqsave(&chan->lock, flags);

  list_for_each_entry_safe(desc, _desc, &chan->active_list, node) {
    dma_async_tx_callback callback;
    void *callback_param;

    if (xdma_desc_status(chan, desc) == DMA_IN_PROGRESS)
      break;

    /* Remove from the list of running transactions */
    list_del(&desc->node);

    /* Run the link descriptor callback function */
    callback = desc->async_tx.callback;
    callback_param = desc->async_tx.callback_param;
    if (callback) {
      spin_unlock_irqrestore(&chan->lock, flags);
      callback(callback_param);
      spin_lock_irqsave(&chan->lock, flags);
    }

    /* Run any dependencies, then free the descriptor */
    dma_run_dependencies(&desc->async_tx);
    dma_pool_free(chan->desc_pool, desc, desc->async_tx.phys);
  }

  spin_unlock_irqrestore(&chan->lock, flags);
}

static enum dma_status xilinx_tx_status(struct dma_chan *dchan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
  struct xdma_chan *chan = to_xilinx_chan(dchan);
  dma_cookie_t last_used;
  dma_cookie_t last_complete;

  xilinx_chan_desc_cleanup(chan);

  last_used = dchan->cookie;
  last_complete = chan->completed_cookie;

  dma_set_tx_state(txstate, last_complete, last_used, 0);

  return dma_async_is_complete(cookie, last_complete, last_used);
}

static int dma_is_running(struct xdma_chan *chan)
{
  return !(DMA_IN(&chan->regs->sr) & XDMA_SR_HALTED_MASK) &&
    (DMA_IN(&chan->regs->cr) & XDMA_CR_RUNSTOP_MASK);
}

static int dma_is_idle(struct xdma_chan *chan)
{
  return DMA_IN(&chan->regs->sr) & XDMA_SR_IDLE_MASK;
}

#define XDMA_DRIVER_DEBUG 1

#if (XDMA_DRIVER_DEBUG == 1)
static void desc_dump(struct xdma_desc_hw *hw)
{
  pr_info("hw desc %x:\n", (unsigned int)hw);
  pr_info("next_desc %x tbuf_addr %x control %x status %x\n", hw->next_desc, hw->buf_addr, hw->control, hw->status);
  //	pr_info("\tbuf_addr %x", hw->buf_addr);
  //	pr_info("\taddr_vsize %x\n", hw->addr_vsize);
  //	pr_info("\thsize %x\n", hw->hsize);
  //	pr_info("\tcontrol %x", hw->control);
  //	pr_info("\tstatus %x\n", hw->status);
}
#endif

/* Stop the hardware, the ongoing transfer will be finished */
static void xdma_halt(struct xdma_chan *chan)
{
  int loop = XDMA_HALT_LOOP;

  DMA_OUT(&chan->regs->cr,
	  DMA_IN(&chan->regs->cr) & ~XDMA_CR_RUNSTOP_MASK);

  /* Wait for the hardware to halt */
  while (loop) {
    if (!(DMA_IN(&chan->regs->cr) & XDMA_CR_RUNSTOP_MASK))
      break;

    loop -= 1;
  }

  if (!loop) {
    pr_debug("Cannot stop channel %x: %x\n",
	     (unsigned int)chan,
	     (unsigned int)DMA_IN(&chan->regs->cr));
    chan->err = 1;
  }

  return;
}

/* Start the hardware. Transfers are not started yet */
static void xdma_start(struct xdma_chan *chan)
{
  int loop = XDMA_HALT_LOOP;

  DMA_OUT(&chan->regs->cr,
	  DMA_IN(&chan->regs->cr) | XDMA_CR_RUNSTOP_MASK);

  /* Wait for the hardware to start */
  while (loop) {
    if (DMA_IN(&chan->regs->cr) & XDMA_CR_RUNSTOP_MASK)
      break;

    loop -= 1;
  }

  if (!loop) {
    pr_debug("Cannot start channel %x: %x\n",
	     (unsigned int)chan,
	     (unsigned int)DMA_IN(&chan->regs->cr));

    chan->err = 1;
  }

  return;
}


static void xdma_start_transfer(struct xdma_chan *chan)
{
  unsigned long flags;
  struct xdma_desc_sw *desch, *desct;
  struct xdma_desc_hw *hw;


  if (chan->err){
    return;
  }

  spin_lock_irqsave(&chan->lock, flags);

  if (list_empty(&chan->pending_list)){
    goto out_unlock;
  }

  /* If hardware is busy, cannot submit */
  if (dma_is_running(chan) && !dma_is_idle(chan)) {
    dev_dbg(chan->dev, "DMA controller still busy\n");
    goto out_unlock;
  }

  /*
   * If hardware is idle, then all descriptors on active list are
   * done, start new transfers
   */
  xdma_halt(chan);

  if (chan->err){
    goto out_unlock;
  }

  if (chan->has_SG) {
    desch = list_first_entry(&chan->pending_list,
			     struct xdma_desc_sw, node);

    desct = container_of(chan->pending_list.prev,
			 struct xdma_desc_sw, node);

    DMA_OUT(&chan->regs->cdr, desch->async_tx.phys);

    xdma_start(chan);

    if (chan->err)
      goto out_unlock;
    list_splice_tail_init(&chan->pending_list, &chan->active_list);

    /* Enable interrupts */
    DMA_OUT(&chan->regs->cr,
	    DMA_IN(&chan->regs->cr) | XDMA_XR_IRQ_ALL_MASK);

    /* Update tail ptr register and start the transfer */
    DMA_OUT(&chan->regs->tdr, desct->async_tx.phys);

    goto out_unlock;
  }
  /* In simple mode */
  xdma_halt(chan);

  if (chan->err)
    goto out_unlock;

  desch = list_first_entry(&chan->pending_list,
			   struct xdma_desc_sw, node);

  list_del(&desch->node);
  list_add_tail(&desch->node, &chan->active_list);

  xdma_start(chan);

  if (chan->err)
    goto out_unlock;

  hw = &desch->hw;

  /* Enable interrupts */
  DMA_OUT(&chan->regs->cr,
	  DMA_IN(&chan->regs->cr) | XDMA_XR_IRQ_ALL_MASK);

  DMA_OUT(&chan->regs->src, hw->buf_addr);

  /* Start the transfer */
  DMA_OUT(&chan->regs->btt_ref,
	  hw->control & XDMA_MAX_TRANS_LEN);

 out_unlock:
  spin_unlock_irqrestore(&chan->lock, flags);
}

static void xdma_issue_pending(struct dma_chan *dchan)
{
  struct xdma_chan *chan = to_xilinx_chan(dchan);

  xdma_start_transfer(chan);
}

/**
 * xdma_update_completed_cookie - Update the completed cookie.
 * @chan : xilinx DMA channel
 *
 * CONTEXT: hardirq
 */
static void xdma_update_completed_cookie(struct xdma_chan *chan)
{
  struct xdma_desc_sw *desc = NULL;
  struct xdma_desc_hw *hw = NULL;
  unsigned long flags;
  dma_cookie_t cookie = -EBUSY;
  int done = 0;

  spin_lock_irqsave(&chan->lock, flags);

  if (list_empty(&chan->active_list)) {
    dev_dbg(chan->dev, "no running descriptors\n");
    goto out_unlock;
  }

  /* Get the last completed descriptor, update the cookie to that */
  list_for_each_entry(desc, &chan->active_list, node) {
    if (chan->has_SG) {
      hw = &desc->hw;

      /* If a BD has no status bits set, hw has it */
      if (!(hw->status & XDMA_BD_STS_ALL_MASK)) {
	break;
      } else {
	done = 1;
	cookie = desc->async_tx.cookie;
      }
    } else {
      /* In non-SG mode, all active entries are done */
      done = 1;
      cookie = desc->async_tx.cookie;
    }
  }

  if (done)
    chan->completed_cookie = cookie;

 out_unlock:
  spin_unlock_irqrestore(&chan->lock, flags);
}

/* Reset hardware */
static int xdma_init(struct xdma_chan *chan)
{
  int loop = XDMA_RESET_LOOP;
  u32 tmp;

  DMA_OUT(&chan->regs->cr,
	  DMA_IN(&chan->regs->cr) | XDMA_CR_RESET_MASK);

  tmp = DMA_IN(&chan->regs->cr) & XDMA_CR_RESET_MASK;

  /* Wait for the hardware to finish reset */
  while (loop && tmp) {
    tmp = DMA_IN(&chan->regs->cr) & XDMA_CR_RESET_MASK;
    loop -= 1;
  }

  if (!loop) {
    dev_err(chan->dev, "reset timeout, cr %x, sr %x\n",
	    DMA_IN(&chan->regs->cr), DMA_IN(&chan->regs->sr));
    return 1;
  }

  return 0;
}


static irqreturn_t xdma_intr_handler(int irq, void *data)
{
  struct xdma_chan *chan = data;
  int update_cookie = 0;
  int to_transfer = 0;
  u32 stat, reg;

  reg = DMA_IN(&chan->regs->cr);

  /* Disable intr */
  DMA_OUT(&chan->regs->cr,
	  reg & ~XDMA_XR_IRQ_ALL_MASK);

  stat = DMA_IN(&chan->regs->sr);
  if (!(stat & XDMA_XR_IRQ_ALL_MASK))
    return IRQ_NONE;

  /* Ack the interrupts */
  DMA_OUT(&chan->regs->sr, XDMA_XR_IRQ_ALL_MASK);

  /* Check for only the interrupts which are enabled */
  stat &= (reg & XDMA_XR_IRQ_ALL_MASK);

  if (stat & XDMA_XR_IRQ_ERROR_MASK) {
    dev_err(chan->dev,
	    "Channel %x has errors %x, cdr %x tdr %x\n",
	    (unsigned int)chan,
	    (unsigned int)DMA_IN(&chan->regs->sr),
	    (unsigned int)DMA_IN(&chan->regs->cdr),
	    (unsigned int)DMA_IN(&chan->regs->tdr));
    chan->err = 1;
  }

  /*
   * Device takes too long to do the transfer when user requires
   * responsiveness
   */
  if (stat & XDMA_XR_IRQ_DELAY_MASK)
    dev_dbg(chan->dev, "Inter-packet latency too long\n");

  if (stat & XDMA_XR_IRQ_IOC_MASK) {
    update_cookie = 1;
    to_transfer = 1;
  }

  if (update_cookie)
    xdma_update_completed_cookie(chan);

  if (to_transfer)
    chan->start_transfer(chan);

  tasklet_schedule(&chan->tasklet);
  return IRQ_HANDLED;
}

static void dma_do_tasklet(unsigned long data)
{
  struct xdma_chan *chan = (struct xdma_chan *)data;

  xilinx_chan_desc_cleanup(chan);
}

/* Append the descriptor list to the pending list */
static void append_desc_queue(struct xdma_chan *chan,
			      struct xdma_desc_sw *desc)
{
  struct xdma_desc_sw *tail =
    container_of(chan->pending_list.prev,
		 struct xdma_desc_sw, node);
  struct xdma_desc_hw *hw;

  if (list_empty(&chan->pending_list))
    goto out_splice;

  /*
   * Add the hardware descriptor to the chain of hardware descriptors
   * that already exists in memory.
   */
  hw = &(tail->hw);
  hw->next_desc = (u32)desc->async_tx.phys;

  /*
   * Add the software descriptor and all children to the list
   * of pending transactions
   */
 out_splice:
  list_splice_tail_init(&desc->tx_list, &chan->pending_list);
}

/*
 * Assign cookie to each descriptor, and append the descriptors to the pending
 * list
 */
static dma_cookie_t xdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
  struct xdma_chan *chan = to_xilinx_chan(tx->chan);
  struct xdma_desc_sw *desc = container_of(tx,
					   struct xdma_desc_sw, async_tx);
  struct xdma_desc_sw *child;
  unsigned long flags;
  dma_cookie_t cookie = -EBUSY;

  if (chan->err) {
    /*
     * If reset fails, need to hard reset the system.
     * Channel is no longer functional
     */
    if (!xdma_init(chan))
      chan->err = 0;
    else
      return cookie;
  }

  spin_lock_irqsave(&chan->lock, flags);
  /*
   * Assign cookies to all of the software descriptors
   * that make up this transaction
   */
  cookie = chan->cookie;
  list_for_each_entry(child, &desc->tx_list, node) {
    cookie++;
    if (cookie < 0)
      cookie = DMA_MIN_COOKIE;

    child->async_tx.cookie = cookie;
  }

  chan->cookie = cookie;

  /* Put this transaction onto the tail of the pending queue */
  append_desc_queue(chan, desc);

  spin_unlock_irqrestore(&chan->lock, flags);

  return cookie;
}

static struct xdma_desc_sw *xdma_alloc_descriptor(
						  struct xdma_chan *chan)
{
  struct xdma_desc_sw *desc;
  dma_addr_t pdesc;

  desc = dma_pool_alloc(chan->desc_pool, GFP_ATOMIC, &pdesc);
  if (!desc) {
    dev_dbg(chan->dev, "out of memory for desc\n");
    return NULL;
  }

  memset(desc, 0, sizeof(*desc));
  INIT_LIST_HEAD(&desc->tx_list);
  dma_async_tx_descriptor_init(&desc->async_tx, &chan->common);
  desc->async_tx.tx_submit = xdma_tx_submit;
  desc->async_tx.phys = pdesc;

  return desc;
}

/**
 * xdma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: DMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xdma_prep_slave_sg(
							  struct dma_chan *dchan, 
							  struct scatterlist *sgl, 
							  unsigned int sg_len,
							  enum dma_transfer_direction direction, 
							  unsigned long flags,
							  void *context)
{
  struct xdma_chan *chan;
  struct xdma_desc_sw *first = NULL, *prev = NULL, *new = NULL;
  struct xdma_desc_hw *hw = NULL, *prev_hw = NULL;

  size_t copy;

  int i;
  struct scatterlist *sg;
  size_t sg_used;
  dma_addr_t dma_src;

#ifdef TEST_DMA_WITH_LOOPBACK
  int total_len;
#endif
  if (!dchan)
    return NULL;

  chan = to_xilinx_chan(dchan);

  if (chan->direction != direction){
    pr_info("error in dma_prep_slave_sg. Don't match channel direction.\n");
    return NULL;
  }

#ifdef TEST_DMA_WITH_LOOPBACK
  total_len = 0;

  for_each_sg(sgl, sg, sg_len, i) {
    total_len += sg_dma_len(sg);
  }
#endif
  /* Build transactions using information in the scatter gather list */
  for_each_sg(sgl, sg, sg_len, i) {
    sg_used = 0;

    /* Loop until the entire scatterlist entry is used */
    while (sg_used < sg_dma_len(sg)) {

      /* Allocate the link descriptor from DMA pool */
      new = xdma_alloc_descriptor(chan);
      if (!new) {
	dev_err(chan->dev,
		"No free memory for link descriptor\n");
	goto fail;
      }

      /*
       * Calculate the maximum number of bytes to transfer,
       * making sure it is less than the hw limit
       */
      copy = min((size_t)(sg_dma_len(sg) - sg_used),
		 (size_t)chan->max_len);
      hw = &(new->hw);

      dma_src = sg_dma_address(sg) + sg_used;

      hw->buf_addr = dma_src;

      /* Fill in the descriptor */
      hw->control = copy;

      /*
       * If this is not the first descriptor, chain the
       * current descriptor after the previous descriptor
       *
       * For the first DMA_MEM_TO_DEV transfer, set SOP
       */
      if (!first) {
	first = new;
	if (direction == DMA_MEM_TO_DEV) {
	  hw->control |= XDMA_BD_SOP;
#ifdef TEST_DMA_WITH_LOOPBACK
	  hw->app_4 = total_len;
#endif
	}
      } else {
	prev_hw = &(prev->hw);
	prev_hw->next_desc = new->async_tx.phys;
      }

      new->async_tx.cookie = 0;
      async_tx_ack(&new->async_tx);
      //      desc_dump(hw);
      prev = new;
      sg_used += copy;

      /* Insert the link descriptor into the LD ring */
      list_add_tail(&new->node, &first->tx_list);
    }
  }

  /* Link the last BD with the first BD */
  hw->next_desc = first->async_tx.phys;

  if (direction == DMA_MEM_TO_DEV)
    hw->control |= XDMA_BD_EOP;

  /* All scatter gather list entries has length == 0 */
  if (!first || !new)
    return NULL;

  new->async_tx.flags = flags;
  new->async_tx.cookie = -EBUSY;

  /* Set EOP to the last link descriptor of new list */
  hw->control |= XDMA_BD_EOP;

  return &first->async_tx;

 fail:
  /*
   * If first was not set, then we failed to allocate the very first
   * descriptor, and we're done
   */
  if (!first)
    return NULL;

  /*
   * First is set, so all of the descriptors we allocated have been added
   * to first->tx_list, INCLUDING "first" itself. Therefore we
   * must traverse the list backwards freeing each descriptor in turn
   */
  xdma_free_desc_list_reverse(chan, &first->tx_list);

  return NULL;
}

/* Run-time device configuration for Axi DMA */
static int xdma_device_control(struct dma_chan *dchan,
			       enum dma_ctrl_cmd cmd, unsigned long arg)
{
  struct xdma_chan *chan;
  unsigned long flags;

  if (!dchan)
    return -EINVAL;

  chan = to_xilinx_chan(dchan);

  if (cmd == DMA_TERMINATE_ALL) {
    /* Halt the DMA engine */
    xdma_halt(chan);

    spin_lock_irqsave(&chan->lock, flags);

    /* Remove and free all of the descriptors in the lists */
    xdma_free_desc_list(chan, &chan->pending_list);
    xdma_free_desc_list(chan, &chan->active_list);

    spin_unlock_irqrestore(&chan->lock, flags);
    return 0;
  } else if (cmd == DMA_SLAVE_CONFIG) {
    /*
     * Configure interrupt coalescing and delay counter
     * Use value XDMA_NO_CHANGE to signal no change
     */
    struct xilinx_dma_config *cfg = (struct xilinx_dma_config *)arg;
    u32 reg = DMA_IN(&chan->regs->cr);

    if (cfg->coalesc <= XDMA_COALESCE_MAX) {
      reg &= ~XDMA_XR_COALESCE_MASK;
      reg |= cfg->coalesc << XDMA_COALESCE_SHIFT;

      chan->config.coalesc = cfg->coalesc;
    }

    if (cfg->delay <= XDMA_DELAY_MAX) {
      reg &= ~XDMA_XR_DELAY_MASK;
      reg |= cfg->delay << XDMA_DELAY_SHIFT;
      chan->config.delay = cfg->delay;
    }

    DMA_OUT(&chan->regs->cr, reg);

    return 0;
  } else
    return -ENXIO;
}

/*
 * Logarithm function to compute alignment shift
 *
 * Only deals with value less than 4096.
 */
static int my_log(int value)
{
  int i = 0;
  while ((1 << i) < value) {
    i++;

    if (i >= 12)
      return 0;
  }

  return i;
}

static void xdma_chan_remove(struct xdma_chan *chan)
{
  free_irq(chan->irq, chan);
  //irq_dispose_mapping(chan->irq);
  list_del(&chan->common.device_node);
  kfree(chan);
}

/*
 * Probing channels
 *
 * . Get channel features from the device tree entry
 * . Initialize special channel handling routines
 */
static int __devinit xdma_chan_probe(struct platform_device * op, struct xdma_device *xdev,
				     int ichan, enum dma_transfer_direction direction,
				     struct device_node * node,
				     u32 feature)
{
  struct xdma_chan *chan;
  int err;
  const __be32 *value;
  u32 width = 0, device_id = 0;
  if(direction == DMA_MEM_TO_DEV){
    dev_info(xdev->dev, "Creating mm2s channel id=%d\n", ichan);
  }else{
    dev_info(xdev->dev, "Creating s2mm channel id=%d\n", ichan);
  }

  /* alloc channel */
  chan = kzalloc(sizeof(*chan), GFP_KERNEL);
  if (!chan) {
    dev_err(xdev->dev, "no free memory for DMA channels!\n");
    err = -ENOMEM;
    goto out_return;
  }

  chan->feature = feature;
  chan->max_len = XDMA_MAX_TRANS_LEN;

  if(direction == DMA_MEM_TO_DEV)
    value = of_get_property(node, "xlnx,include-mm2s-dre", NULL);
  else
    value = of_get_property(node, "xlnx,include-s2mm-dre", NULL);

  if (value)
    chan->has_DRE = be32_to_cpup(value);

  if(direction == DMA_MEM_TO_DEV)
    value = of_get_property(node, "xlnx,mm2s-data-width", NULL);
  else
    value = of_get_property(node, "xlnx,s2mm-data-width", NULL);

  if (value) {
    width = be32_to_cpup(value) >> 3; /* convert bits to bytes */

    /* If data width is greater than 8 bytes, DRE is not in hw */
    if (width > 8)
      chan->has_DRE = 0;

    chan->feature |= width - 1;
  }

  value = of_get_property(node, "xlnx,device-id", NULL);
  if (value)
    device_id = be32_to_cpup(value);

  if (feature & XILINX_DMA_IP_DMA) {
    chan->has_SG = (xdev->feature & XDMA_FTR_HAS_SG) >>
      XDMA_FTR_HAS_SG_SHIFT;

    chan->start_transfer = xdma_start_transfer;

    chan->direction = direction;
  }

  chan->regs = (struct xdma_regs *)xdev->regs;

  if (chan->direction == DMA_DEV_TO_MEM) {
    chan->regs = (struct xdma_regs *)((u32)xdev->regs +
				      XDMA_RX_CHANNEL_OFFSET);
  }
  chan->id = ichan;

  /*
   * Used by dmatest channel matching in slave transfers
   * Can change it to be a structure to have more matching information
   */
  chan->private = (chan->direction & 0xFF) |
    (chan->feature & XILINX_DMA_IP_MASK) |
    (device_id << XDMA_DEVICE_ID_SHIFT);
  chan->common.private = (void *)&(chan->private);

  if (!chan->has_DRE)
    xdev->common.copy_align = my_log(width);

  chan->dev = xdev->dev;
  xdev->chan[chan->id] = chan;

  tasklet_init(&chan->tasklet, dma_do_tasklet, (unsigned long)chan);

  /* Initialize the channel */
  if (xdma_init(chan)) {
    dev_err(xdev->dev, "Reset channel failed\n");
    goto out_free_chan;
  }


  spin_lock_init(&chan->lock);
  INIT_LIST_HEAD(&chan->pending_list);
  INIT_LIST_HEAD(&chan->active_list);

  chan->common.device = &xdev->common;

  /* find the IRQ line, if it exists in the device tree */
  // I assume the first IRQ is for mem to device operation.
  {
    struct resource * r_irq; 
    if(direction == DMA_MEM_TO_DEV){
      r_irq = platform_get_resource(op, IORESOURCE_IRQ, 0);
      //chan->irq = irq_of_parse_and_map(node, 0);
    }else{
      r_irq = platform_get_resource(op, IORESOURCE_IRQ, 1);
      //chan->irq = irq_of_parse_and_map(node, 1);
    }

    if(!r_irq){
      goto out_free_irq;
    }

    chan->irq = r_irq->start;
    dev_info(xdev->dev, "\t irq=%d\n", chan->irq);
	
    err = request_irq(chan->irq, xdma_intr_handler, IRQF_SHARED,
		      "xilinx-dma-controller", chan);
    if (err) {
      dev_err(xdev->dev, "unable to request IRQ\n");
      goto out_free_irq;
    }
  }
	
  /* Add the channel to DMA device channel list */
  list_add_tail(&chan->common.device_node, &xdev->common.channels);
  xdev->common.chancnt++;

  return 0;

 out_free_irq:
  free_irq(chan->irq, chan);

 out_free_chan:
  kfree(chan);
 out_return:
  return err;
}

static int __devinit xdma_of_probe(struct platform_device *op)
{
  struct xdma_device *xdev;
  struct device_node /**child,*/ *node;
  int ichan; // loop variable for channel generation
  int err;
  const __be32 *value;

  dev_info(&op->dev, "Probing xilinx axi dma engine\n");

  xdev = kzalloc(sizeof(struct xdma_device), GFP_KERNEL);
  if (!xdev) {
    dev_err(&op->dev, "Not enough memory for device\n");
    err = -ENOMEM;
    goto out_return;
  }

  xdev->dev = &(op->dev);
  INIT_LIST_HEAD(&xdev->common.channels);

  node = op->dev.of_node;
  xdev->feature = 0;

  /* iomap registers */
  xdev->regs = of_iomap(node, 0);
  if (!xdev->regs) {
    dev_err(&op->dev, "unable to iomap registers\n");
    err = -ENOMEM;
    goto out_free_xdev;
  }

  /*
   * Axi DMA only do slave transfers
   */
  if (of_device_is_compatible(node, "xlnx,axi-dma-6.03.a")) {

    xdev->feature |= XILINX_DMA_IP_DMA;
    value = of_get_property(node,
			    "xlnx,sg-include-stscntrl-strm",
			    NULL);
    if (value) {
      if (be32_to_cpup(value) == 1) {
	xdev->feature |= (XDMA_FTR_STSCNTRL_STRM |
			  XDMA_FTR_HAS_SG);
      }
    }

    value = of_get_property(node,
			    "xlnx,include-sg",
			    NULL);
    if(value){
      if(be32_to_cpup(value) == 1){
	xdev->feature |= XDMA_FTR_HAS_SG;
      }
    }

    dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
    dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);
    xdev->common.device_prep_slave_sg = xdma_prep_slave_sg;
    xdev->common.device_control = xdma_device_control;
    xdev->common.device_issue_pending = xdma_issue_pending;
		
    /* modified by yohei matsumoto */
    // following codes load channel count.
    value = of_get_property(node, 
			    "xlnx,include-mm2s",
			    NULL);
    xdev->has_mm2s = 0;
    if(value){
      if(be32_to_cpup(value) == 1){
	xdev->has_mm2s = 1;
      }
    }  		
		
    xdev->num_mm2s_chans = 0;
    value = of_get_property(node, 
			    "xlnx,num-mm2s-channels",
			    NULL);
    if(value){
      xdev->num_mm2s_chans = be32_to_cpup(value);
    }

    value = of_get_property(node, 
			    "xlnx,include-s2mm",
			    NULL);
    xdev->has_s2mm = 0;
    if(value){
      if(be32_to_cpup(value) == 1){
	xdev->has_s2mm = 1;
      }
    }

    xdev->num_s2mm_chans = 0;
    value = of_get_property(node,
			    "xlnx,num-s2mm-channels",
			    NULL);
    if(value){
      xdev->num_s2mm_chans = be32_to_cpup(value);
    }
   
    ichan = 0;
    xdev->chan = kzalloc(sizeof(struct xdma_chan**) 
			 * (xdev->num_s2mm_chans + xdev->num_mm2s_chans), 
			 GFP_KERNEL);
    if(!xdev->chan){
      goto out_free_chan;
    }

    for(; ichan < xdev->num_mm2s_chans; ichan++){
      // probe mm2s channel
      err = xdma_chan_probe(op, xdev, ichan, DMA_MEM_TO_DEV, node, xdev->feature);
      if(err){
	goto out_free_chan;
      }
    }

    for(;ichan < xdev->num_s2mm_chans + xdev->num_mm2s_chans; ichan++){
      // probe s2mm channel
      err = xdma_chan_probe(op, xdev, ichan, DMA_DEV_TO_MEM, node, xdev->feature);
      if(err){
	goto out_free_chan;
      }
    }
  }

  xdev->common.device_alloc_chan_resources =
    xdma_alloc_chan_resources;
  xdev->common.device_free_chan_resources =
    xdma_free_chan_resources;
  xdev->common.device_tx_status = xilinx_tx_status;
  xdev->common.dev = &op->dev;

  dev_set_drvdata(&op->dev, xdev);
  /*
    for_each_child_of_node(node, child) {
    xdma_chan_probe(xdev, child, xdev->feature);
    }
  */
  dma_async_device_register(&xdev->common);

  return 0;

 out_free_chan:
  for(;ichan>=0; ichan--){
    xdma_chan_remove(xdev->chan[ichan]);
  }
  kfree(xdev->chan);

 out_free_xdev:
  kfree(xdev);

 out_return:
  return err;
}

static int __devexit xdma_of_remove(struct platform_device *op)
{
  struct xdma_device *xdev;
  int i;

  xdev = dev_get_drvdata(&op->dev);
  dma_async_device_unregister(&xdev->common);

  for (i = 0; i < xdev->num_s2mm_chans + xdev->num_mm2s_chans; i++) {
    if (xdev->chan[i]){
      dev_info(xdev->dev, "removing channel %d irq %d\n", i, xdev->chan[i]->irq);
      xdma_chan_remove(xdev->chan[i]);
    }
  }

  iounmap(xdev->regs);
  dev_set_drvdata(&op->dev, NULL);
  kfree(xdev->chan);
  kfree(xdev);

  return 0;
}

static const struct of_device_id xdma_of_ids[] = {
  { .compatible = "xlnx,axi-dma-6.03.a",},
  { .compatible = "xlnx,axi-dma-1.00.a",},
  {}
};

static struct platform_driver xdma_of_driver = {
  .driver = {
    .name = "xilinx-dma",
    .owner = THIS_MODULE,
    .of_match_table = xdma_of_ids,
  },
  .probe = xdma_of_probe,
  .remove = __devexit_p(xdma_of_remove),
};

module_platform_driver(xdma_of_driver);

MODULE_AUTHOR("Yohei Matsumoto");
MODULE_DESCRIPTION("Xilinx DMA driver");
MODULE_LICENSE("GPL v2");
