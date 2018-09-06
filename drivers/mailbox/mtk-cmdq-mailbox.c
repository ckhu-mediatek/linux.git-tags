// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/mtk-cmdq-mailbox.h>
#include <linux/of_device.h>

#define CMDQ_IRQ_MASK			0xffff

#define CMDQ_CURR_IRQ_STATUS		0x10
#define CMDQ_THR_SLOT_CYCLES		0x30
#define CMDQ_THR_BASE			0x100
#define CMDQ_THR_SIZE			0x80
#define CMDQ_THR_WARM_RESET		0x00
#define CMDQ_THR_ENABLE_TASK		0x04
#define CMDQ_THR_SUSPEND_TASK		0x08
#define CMDQ_THR_CURR_STATUS		0x0c
#define CMDQ_THR_IRQ_STATUS		0x10
#define CMDQ_THR_IRQ_ENABLE		0x14
#define CMDQ_THR_CURR_ADDR		0x20
#define CMDQ_THR_END_ADDR		0x24
#define CMDQ_THR_WAIT_TOKEN		0x30
#define CMDQ_THR_PRIORITY		0x40

#define CMDQ_THR_ACTIVE_SLOT_CYCLES	0x3200
#define CMDQ_THR_ENABLED		0x1
#define CMDQ_THR_DISABLED		0x0
#define CMDQ_THR_SUSPEND		0x1
#define CMDQ_THR_RESUME			0x0
#define CMDQ_THR_STATUS_SUSPENDED	BIT(1)
#define CMDQ_THR_DO_WARM_RESET		BIT(0)
#define CMDQ_THR_IRQ_DONE		0x1
#define CMDQ_THR_IRQ_ERROR		0x12
#define CMDQ_THR_IRQ_EN			(CMDQ_THR_IRQ_ERROR | CMDQ_THR_IRQ_DONE)
#define CMDQ_THR_IS_WAITING		BIT(31)

struct cmdq_thread {
	struct mbox_chan	*chan;
	void __iomem		*base;
	u32			priority;
	struct cmdq_pkt		*pkt; /* the packet sent from mailbox client */
};

struct cmdq {
	struct mbox_controller	mbox;
	void __iomem		*base;
	u32			irq;
	u32			thread_nr;
	struct cmdq_thread	*thread;
	struct clk		*clock;
	bool			suspended;
};

static int cmdq_thread_suspend(struct cmdq *cmdq, struct cmdq_thread *thread)
{
	u32 status;

	writel(CMDQ_THR_SUSPEND, thread->base + CMDQ_THR_SUSPEND_TASK);

	/* If already disabled, treat as suspended successful. */
	if (!(readl(thread->base + CMDQ_THR_ENABLE_TASK) & CMDQ_THR_ENABLED))
		return 0;

	if (readl_poll_timeout_atomic(thread->base + CMDQ_THR_CURR_STATUS,
			status, status & CMDQ_THR_STATUS_SUSPENDED, 0, 10)) {
		dev_err(cmdq->mbox.dev, "suspend GCE thread 0x%x failed\n",
			(u32)(thread->base - cmdq->base));
		return -EFAULT;
	}

	return 0;
}

static void cmdq_thread_resume(struct cmdq_thread *thread)
{
	writel(CMDQ_THR_RESUME, thread->base + CMDQ_THR_SUSPEND_TASK);
}

static void cmdq_init(struct cmdq *cmdq)
{
	WARN_ON(clk_enable(cmdq->clock) < 0);
	writel(CMDQ_THR_ACTIVE_SLOT_CYCLES, cmdq->base + CMDQ_THR_SLOT_CYCLES);
	clk_disable(cmdq->clock);
}

static int cmdq_thread_reset(struct cmdq *cmdq, struct cmdq_thread *thread)
{
	u32 warm_reset;

	writel(CMDQ_THR_DO_WARM_RESET, thread->base + CMDQ_THR_WARM_RESET);
	if (readl_poll_timeout_atomic(thread->base + CMDQ_THR_WARM_RESET,
			warm_reset, !(warm_reset & CMDQ_THR_DO_WARM_RESET),
			0, 10)) {
		dev_err(cmdq->mbox.dev, "reset GCE thread 0x%x failed\n",
			(u32)(thread->base - cmdq->base));
		return -EFAULT;
	}

	return 0;
}

static void cmdq_thread_disable(struct cmdq *cmdq, struct cmdq_thread *thread)
{
	cmdq_thread_reset(cmdq, thread);
	writel(CMDQ_THR_DISABLED, thread->base + CMDQ_THR_ENABLE_TASK);
}

static bool cmdq_thread_is_in_wfe(struct cmdq_thread *thread)
{
	return readl(thread->base + CMDQ_THR_WAIT_TOKEN) & CMDQ_THR_IS_WAITING;
}

static void cmdq_thread_irq_handler(struct cmdq *cmdq,
				    struct cmdq_thread *thread)
{
	unsigned long flags;
	u32 curr_pa, irq_flag, end_pa;
	int ret = 0;

	spin_lock_irqsave(&thread->chan->lock, flags);
	irq_flag = readl(thread->base + CMDQ_THR_IRQ_STATUS);
	writel(~irq_flag, thread->base + CMDQ_THR_IRQ_STATUS);

	curr_pa = readl(thread->base + CMDQ_THR_CURR_ADDR);
	end_pa = readl(thread->base + CMDQ_THR_END_ADDR);

	if (curr_pa != end_pa ||  irq_flag & CMDQ_THR_IRQ_ERROR)
		ret = -EFAULT;

	thread->pkt = NULL;
	cmdq_thread_disable(cmdq, thread);
	clk_disable(cmdq->clock);
	spin_unlock_irqrestore(&thread->chan->lock, flags);
	mbox_chan_txdone(thread->chan, ret);
}

static irqreturn_t cmdq_irq_handler(int irq, void *dev)
{
	struct cmdq *cmdq = dev;
	unsigned long irq_status;
	int bit;

	irq_status = readl(cmdq->base + CMDQ_CURR_IRQ_STATUS) & CMDQ_IRQ_MASK;
	if (!(irq_status ^ CMDQ_IRQ_MASK))
		return IRQ_NONE;

	for_each_clear_bit(bit, &irq_status, fls(CMDQ_IRQ_MASK))
		cmdq_thread_irq_handler(cmdq, &cmdq->thread[bit]);

	return IRQ_HANDLED;
}

static int cmdq_suspend(struct device *dev)
{
	struct cmdq *cmdq = dev_get_drvdata(dev);
	struct cmdq_thread *thread;
	int i;
	bool task_running = false;

	cmdq->suspended = true;

	for (i = 0; i < cmdq->thread_nr; i++) {
		thread = &cmdq->thread[i];
		if (thread->pkt) {
			task_running = true;
			break;
		}
	}

	if (task_running)
		dev_warn(dev, "exist running task(s) in suspend\n");

	clk_unprepare(cmdq->clock);

	return 0;
}

static int cmdq_resume(struct device *dev)
{
	struct cmdq *cmdq = dev_get_drvdata(dev);

	WARN_ON(clk_prepare(cmdq->clock) < 0);
	cmdq->suspended = false;
	return 0;
}

static int cmdq_remove(struct platform_device *pdev)
{
	struct cmdq *cmdq = platform_get_drvdata(pdev);

	clk_unprepare(cmdq->clock);

	return 0;
}

static int cmdq_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct cmdq_pkt *pkt = (struct cmdq_pkt *)data;
	struct cmdq_thread *thread = (struct cmdq_thread *)chan->con_priv;
	struct cmdq *cmdq = dev_get_drvdata(chan->mbox->dev);

	/* Client should not flush new tasks if suspended. */
	WARN_ON(cmdq->suspended);

	thread->pkt = pkt;

	WARN_ON(clk_enable(cmdq->clock) < 0);
	WARN_ON(cmdq_thread_reset(cmdq, thread) < 0);

	writel(thread->pkt->pa_base, thread->base + CMDQ_THR_CURR_ADDR);
	writel(thread->pkt->pa_base + pkt->cmd_buf_size,
	       thread->base + CMDQ_THR_END_ADDR);
	writel(thread->priority, thread->base + CMDQ_THR_PRIORITY);
	writel(CMDQ_THR_IRQ_EN, thread->base + CMDQ_THR_IRQ_ENABLE);
	writel(CMDQ_THR_ENABLED, thread->base + CMDQ_THR_ENABLE_TASK);

	return 0;
}

static void cmdq_mbox_abort_data(struct mbox_chan *chan)
{
	struct cmdq_thread *thread = (struct cmdq_thread *)chan->con_priv;
	struct cmdq *cmdq = dev_get_drvdata(chan->mbox->dev);
	unsigned long flags;
	u32 enable;

	spin_lock_irqsave(&thread->chan->lock, flags);
	if (!thread->pkt)
		goto out;

	WARN_ON(cmdq_thread_suspend(cmdq, thread) < 0);
	if (!cmdq_thread_is_in_wfe(thread))
		goto wait;

	thread->pkt = NULL;

	cmdq_thread_resume(thread);
	cmdq_thread_disable(cmdq, thread);
	clk_disable(cmdq->clock);

out:
	spin_unlock_irqrestore(&thread->chan->lock, flags);
	return;

wait:
	cmdq_thread_resume(thread);
	spin_unlock_irqrestore(&thread->chan->lock, flags);
	if (readl_poll_timeout_atomic(thread->base + CMDQ_THR_ENABLE_TASK,
				      enable, !enable, 1, 20))
		dev_err(cmdq->mbox.dev, "Fail to wait GCE thread 0x%x done\n",
			(u32)(thread->base - cmdq->base));
}

static int cmdq_mbox_startup(struct mbox_chan *chan)
{
	return 0;
}

static void cmdq_mbox_shutdown(struct mbox_chan *chan)
{
}

static const struct mbox_chan_ops cmdq_mbox_chan_ops = {
	.abort_data = cmdq_mbox_abort_data,
	.send_data = cmdq_mbox_send_data,
	.startup = cmdq_mbox_startup,
	.shutdown = cmdq_mbox_shutdown,
};

static struct mbox_chan *cmdq_xlate(struct mbox_controller *mbox,
		const struct of_phandle_args *sp)
{
	int ind = sp->args[0];
	struct cmdq_thread *thread;

	if (ind >= mbox->num_chans)
		return ERR_PTR(-EINVAL);

	thread = (struct cmdq_thread *)mbox->chans[ind].con_priv;
	thread->priority = sp->args[1];
	thread->chan = &mbox->chans[ind];

	return &mbox->chans[ind];
}

static int cmdq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct cmdq *cmdq;
	int err, i;

	cmdq = devm_kzalloc(dev, sizeof(*cmdq), GFP_KERNEL);
	if (!cmdq)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cmdq->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cmdq->base)) {
		dev_err(dev, "failed to ioremap gce\n");
		return PTR_ERR(cmdq->base);
	}

	cmdq->irq = platform_get_irq(pdev, 0);
	if (!cmdq->irq) {
		dev_err(dev, "failed to get irq\n");
		return -EINVAL;
	}
	err = devm_request_irq(dev, cmdq->irq, cmdq_irq_handler, IRQF_SHARED,
			       "mtk_cmdq", cmdq);
	if (err < 0) {
		dev_err(dev, "failed to register ISR (%d)\n", err);
		return err;
	}

	dev_dbg(dev, "cmdq device: addr:0x%p, va:0x%p, irq:%d\n",
		dev, cmdq->base, cmdq->irq);

	cmdq->clock = devm_clk_get(dev, "gce");
	if (IS_ERR(cmdq->clock)) {
		dev_err(dev, "failed to get gce clk\n");
		return PTR_ERR(cmdq->clock);
	}

	cmdq->thread_nr = (u32)(unsigned long)of_device_get_match_data(dev);
	cmdq->mbox.dev = dev;
	cmdq->mbox.chans = devm_kcalloc(dev, cmdq->thread_nr,
					sizeof(*cmdq->mbox.chans), GFP_KERNEL);
	if (!cmdq->mbox.chans)
		return -ENOMEM;

	cmdq->mbox.num_chans = cmdq->thread_nr;
	cmdq->mbox.ops = &cmdq_mbox_chan_ops;
	cmdq->mbox.of_xlate = cmdq_xlate;

	cmdq->mbox.txdone_irq = true;
	cmdq->mbox.txdone_poll = false;

	cmdq->thread = devm_kcalloc(dev, cmdq->thread_nr,
					sizeof(*cmdq->thread), GFP_KERNEL);
	if (!cmdq->thread)
		return -ENOMEM;

	for (i = 0; i < cmdq->thread_nr; i++) {
		cmdq->thread[i].base = cmdq->base + CMDQ_THR_BASE +
				CMDQ_THR_SIZE * i;
		cmdq->mbox.chans[i].con_priv = (void *)&cmdq->thread[i];
	}

	err = devm_mbox_controller_register(dev, &cmdq->mbox);
	if (err < 0) {
		dev_err(dev, "failed to register mailbox: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, cmdq);
	WARN_ON(clk_prepare(cmdq->clock) < 0);

	cmdq_init(cmdq);

	return 0;
}

static const struct dev_pm_ops cmdq_pm_ops = {
	.suspend = cmdq_suspend,
	.resume = cmdq_resume,
};

static const struct of_device_id cmdq_of_ids[] = {
	{.compatible = "mediatek,mt8173-gce", .data = (void *)16},
	{}
};

static struct platform_driver cmdq_drv = {
	.probe = cmdq_probe,
	.remove = cmdq_remove,
	.driver = {
		.name = "mtk_cmdq",
		.pm = &cmdq_pm_ops,
		.of_match_table = cmdq_of_ids,
	}
};

static int __init cmdq_drv_init(void)
{
	return platform_driver_register(&cmdq_drv);
}

static void __exit cmdq_drv_exit(void)
{
	platform_driver_unregister(&cmdq_drv);
}

subsys_initcall(cmdq_drv_init);
module_exit(cmdq_drv_exit);

MODULE_LICENSE("GPL v2");
