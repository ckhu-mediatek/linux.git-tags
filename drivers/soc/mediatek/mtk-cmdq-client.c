// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2019 MediaTek Inc.

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/soc/mediatek/mtk-cmdq.h>

#define CMDQ_EOC_IRQ_EN		BIT(0)
#define CMD_BUF_SIZE 1024

struct mtk_cmdq_client {
	struct clk *clk;
	struct mbox_client client;
	struct mbox_chan *chan;
	struct completion c;
};

struct cmdq_setting {
	u32 value;
	u32 subsys;
	u32 offset;
};

static struct cmdq_pkt *mtk_cmdq_pkt_create(struct device *dev, size_t size)
{
	struct cmdq_pkt *pkt;
	
	dma_addr_t dma_addr;

	pkt = kzalloc(sizeof(*pkt), GFP_KERNEL);
	if (!pkt)
		return ERR_PTR(-ENOMEM);
	pkt->va_base = kzalloc(size, GFP_KERNEL);
	if (!pkt->va_base) {
		kfree(pkt);
		return ERR_PTR(-ENOMEM);
	}
	pkt->buf_size = size;

	dma_addr = dma_map_single(dev, pkt->va_base, pkt->buf_size,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		dev_err(dev, "dma map failed, size=%u\n", (u32)(u64)size);
		kfree(pkt->va_base);
		kfree(pkt);
		return ERR_PTR(-ENOMEM);
	}

	pkt->pa_base = dma_addr;

	return pkt;
}

static void mtk_cmdq_pkt_destroy(struct device *dev, struct cmdq_pkt *pkt)
{
	dma_unmap_single(dev, pkt->pa_base, pkt->buf_size,
			 DMA_TO_DEVICE);
	kfree(pkt->va_base);
	kfree(pkt);
}

static int cmdq_pkt_append_command(struct cmdq_pkt *pkt, enum cmdq_code code,
				   u32 arg_a, u32 arg_b)
{
	return 0;
}

static int cmdq_pkt_eoc(struct cmdq_pkt *pkt)
{
	/* insert EOC and generate IRQ for each command iteration */
	return cmdq_pkt_append_command(pkt, CMDQ_CODE_EOC, 0, CMDQ_EOC_IRQ_EN);
}

static void pkt_sent(struct mbox_client *cl, void *mssg, int r)
{
	struct mtk_cmdq_client *client = container_of(cl,
			struct mtk_cmdq_client, client);

	mtk_cmdq_pkt_destroy(client->client.dev, (struct cmdq_pkt *)mssg);
		
	complete(&client->c);
}

void mtk_cmdq_client_async(struct device *dev, int setting_nr,
			   struct cmdq_setting *settings)
{
	struct mtk_cmdq_client *client = dev_get_drvdata(dev);
	struct cmdq_pkt *pkt;
	int ret;
	int i;

	for (i = 0; i < setting_nr; i++) {
		pkt = mtk_cmdq_pkt_create(dev, CMD_BUF_SIZE);
		cmdq_pkt_write(pkt, settings[i].value, settings[i].subsys,
			       settings[i].offset);
		cmdq_pkt_eoc(pkt);
		dma_sync_single_for_device(dev, pkt->pa_base,
					   pkt->cmd_buf_size, DMA_TO_DEVICE);
		mbox_send_message(client->chan, pkt);
	}

	for (i = 0; i < 10; i++) {
		ret = wait_for_completion_timeout(&client->c,
						  msecs_to_jiffies(70));
		if (ret <= 0)
			dev_err(dev, "wait for pkt sent timed out\n");
	}
}

void mtk_cmdq_client_sync(struct device *dev, int setting_nr,
			       struct cmdq_setting *settings)
{
	struct mtk_cmdq_client *client = dev_get_drvdata(dev);
	struct cmdq_pkt *pkt;
	int ret;
	int i;

	pkt = mtk_cmdq_pkt_create(dev, CMD_BUF_SIZE);

	for (i = 0; i < setting_nr; i++) {
		pkt->cmd_buf_size = 0;
		cmdq_pkt_write(pkt, settings[i].value, settings[i].subsys,
			       settings[i].offset);		
		cmdq_pkt_eoc(pkt);
		dma_sync_single_for_device(dev, pkt->pa_base,
					   pkt->cmd_buf_size, DMA_TO_DEVICE);
		ret = mbox_send_message(client->chan, pkt);
		if (ret == -ETIME)
			dev_err(dev, "wait for pkt sent timed out\n");
	}

	mtk_cmdq_pkt_destroy(dev, pkt);
}

int mtk_cmdq_client_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_cmdq_client *client;

	client = devm_kzalloc(dev, sizeof(*client), GFP_KERNEL);

	client->clk = of_clk_get(dev->of_node, 0);

	client->client.dev = dev;
	client->client.tx_block = false;
	if (client->client.tx_block)
		client->client.tx_tout = 70;
	else 
		client->client.tx_done = pkt_sent;

	init_completion(&client->c);
	client->chan = mbox_request_channel(&client->client, 0);

	platform_set_drvdata(pdev, client);

	return 0;
}

MODULE_LICENSE("GPL v2");
