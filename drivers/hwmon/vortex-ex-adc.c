/*
 * DMP Vortex Ex ADC driver
 *
 * Copyright 2014 Jan Cornelis
 *
 * Licensed under the GPL-2.
 */
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/hwmon.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

#include <linux/pci.h>

#define DRV_NAME "vortex_adc"
#define PCI_VENDOR_ID_DMP	0x17F3
#define PCI_DEVICE_ID_VORTEXEX_SB 0x6011

#define DEV_SB	7
#define VORTEX_EX_SB_FN0	0
#define VORTEX_EX_SB_FN1	1

#define VORTEX_ADC_PHYS_ADDR	0xfe00
#define VORTEX_ADC_LEN		(4 * sizeof(u32))

#define VORTEX_ADC_ENABLE_OFFSET	20
#define VORTEX_ADC_IN7_SELECT		21
	#define IN7_ADC			0
	#define IN7_TEMP		1
#define VORTEX_ADC_TEMP_LOCATION_OFFSET	22
	#define TEMP_SB			0
	#define TEMP_CPU		1

#define FN1_REG_ADC	0xE0
#define FN1_REG_8051A	0xDE
#define FN0_REG_ENABLE	0xBC

#define ENABLE_ADC_BIT	28


#define FN0_REG_ISA_CONFIG	0xC0


/* ADC peripheral definitions */
#define ADC_SEL_REG_OFFSET	0

#define ADC_CTRL_REG_OFFSET	1
	#define ADC_CTRL_AST_OFFSET	0
	#define ADC_CTRL_ASM_OFFSET	1
	#define ADC_CTRL_AICS_OFFSET	2
	#define ADC_CTRL_APM_OFFSET	3
	#define ADC_CTRL_IIT_OFFSET	4
	#define ADC_CTRL_IMC_OFFSET	7

#define ADC_STATUS_REG_OFFSET	2
	#define ADC_STATUS_DR_OFFSET	0
	#define ADC_STATUS_IS_OFFSET	7

#define ADC_DATA_REG_OFFSET	4
	#define ADC_DATA_AOD_OFFSET	0
	#define ADC_DATA_AC_OFFSET	13

#define NUM_INPUTS	8

struct vortex_adc_data {
	struct mutex lock;
	const char *name;
	struct device *vortex_ex_adc_hwmon;
	wait_queue_head_t wq;
	unsigned short base_address;
	u16 raw[NUM_INPUTS];
};

static int check(int ba)
{
	int rv = inb(ba + ADC_STATUS_REG_OFFSET) &
		(1 << ADC_DATA_AOD_OFFSET);
	return rv;
}


static struct vortex_adc_data *vortex_adc_conversion(struct device *dev,
						     u8 channel)
{
	struct vortex_adc_data *data = dev_get_drvdata(dev);
	int rv = 0;
	u16 raw;
	u16 dummy;

	dev_dbg(dev, "Channel: %d\n", channel);
	dev_dbg(dev, "address: %x\n", data->base_address);

	if (channel > 8)
		return NULL;

	rv = mutex_lock_killable(&data->lock);
	if (rv < 0)
		return NULL;

	/* Disable ADC */
	outb_p((1 << ADC_CTRL_APM_OFFSET),
	       data->base_address + ADC_CTRL_REG_OFFSET);

	/* Empty Fifo*/
	while ((inb(data->base_address + ADC_STATUS_REG_OFFSET) & 0x01) != 0)
		dummy = inw_p(data->base_address + ADC_DATA_REG_OFFSET);

	/* Enable Channel */
	outb_p((1 << channel),
	       data->base_address + ADC_SEL_REG_OFFSET);


	/* Start Conversion */
	outb_p((1 << ADC_CTRL_AST_OFFSET),
	       data->base_address + ADC_CTRL_REG_OFFSET);

	/*
	 * Polling based driver. No sleep allowed because there is no event
	 * that wakes up the driver again
	 */
	while (!check(data->base_address))
		usleep_range(usecs_to_jiffies(400), usecs_to_jiffies(500));

	if (rv < 0) {
		dev_dbg(dev, "Wait interrupted");
		goto unlock;
	}

	/* Get Raw data*/
	raw = inw(data->base_address + ADC_DATA_REG_OFFSET);
	data->raw[channel] = raw & 0x7FF;

	mutex_unlock(&data->lock);
	return data;
unlock:
	mutex_unlock(&data->lock);

	return NULL;
}


static ssize_t vortex_adc_read(struct device *dev, struct device_attribute
			       *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct vortex_adc_data *data = vortex_adc_conversion(dev, attr->index);

	if (NULL == data)
		return 0;

	return sprintf(buf, "%d\n", data->raw[attr->index]);
}


static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, vortex_adc_read, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, vortex_adc_read, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, vortex_adc_read, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, vortex_adc_read, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, vortex_adc_read, NULL, 4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, vortex_adc_read, NULL, 5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, vortex_adc_read, NULL, 6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, vortex_adc_read, NULL, 7);

static struct attribute *analog_in_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(analog_in);


/*
 *
 */
static const struct pci_device_id vortex_ex_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_DMP, PCI_DEVICE_ID_VORTEXEX_SB), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, vortex_ex_pci_tbl);


static int pci_readmodifywrite_config_dword(struct pci_dev *pdev, int fn,
					    int reg, u32 mask, u32 val)
{
	int err;
	u32 regval;

	err = pci_bus_read_config_dword(pdev->bus, fn, reg, &regval);
	if (err < 0)
		return err;
	regval = (regval & ~mask) | (val & mask);

	return pci_bus_write_config_dword(pdev->bus, fn, reg, regval);
}

static int pci_readmodifywrite_config_word(struct pci_dev *pdev, int fn,
					   int reg, u16 mask, u16 val)
{
	int err;
	u16 regval;

	err = pci_bus_read_config_word(pdev->bus, fn, reg, &regval);
	if (err < 0)
		return err;
	regval = (regval & ~mask) | (val & mask);

	return pci_bus_write_config_word(pdev->bus, fn, reg, regval);
}

static int vortex_adc_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	void *phys_addr;
	int err;

	struct vortex_adc_data *data;

	err = pci_enable_device(pdev);
	if (err)
		return err;

	if ((pdev->devfn & 0x07) != 1) {
		err = -ENODEV;
		goto failure_disable_pci;
	}

	/* Force Highspeed ISA */
	pci_readmodifywrite_config_dword(pdev,
					 (DEV_SB << 3) | VORTEX_EX_SB_FN0,
					 FN0_REG_ISA_CONFIG,
					 0x8000C000L,
					 0x8000C000L);

	/* Enable ADC (configuration on South Bridge Function 0 */
	pci_readmodifywrite_config_dword(pdev,
					 (DEV_SB << 3) | VORTEX_EX_SB_FN0,
					 FN0_REG_ENABLE,
					 (1L << ENABLE_ADC_BIT),
					 0);

	/* Config 8051A*/
	pci_readmodifywrite_config_word(pdev,
					(DEV_SB << 3) | VORTEX_EX_SB_FN1,
					FN1_REG_8051A, (1 << 1), (1 << 1));

	/* Request IO region */
	phys_addr = request_region(VORTEX_ADC_PHYS_ADDR, VORTEX_ADC_LEN,
				   DRV_NAME);
	if (phys_addr == NULL)
		goto failure_disable_pci;

	/* Configure PCI registers */
	err = pci_write_config_dword(pdev, FN1_REG_ADC,
		VORTEX_ADC_PHYS_ADDR | (1L << VORTEX_ADC_ENABLE_OFFSET));
	if (err < 0) {
		dev_dbg(&pdev->dev, "Write physical address failed\n");
		err = -ENODEV;
		goto failure_request_mem_region;
	}

	dev_dbg(&pdev->dev, "Vendor: %04x; Device: %04x; fn: %02x",
		pdev->vendor, pdev->device, pdev->devfn);

	dev_dbg(&pdev->dev, "IO mapped addr: %x\n", VORTEX_ADC_PHYS_ADDR);

	data = devm_kzalloc(&pdev->dev, sizeof(struct vortex_adc_data),
			    GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		dev_dbg(&pdev->dev, "private data allocation failed\n");
		goto failure_request_mem_region;
	}

	pci_set_drvdata(pdev, data);
	data->base_address = VORTEX_ADC_PHYS_ADDR;
	mutex_init(&data->lock);
	init_waitqueue_head(&data->wq);

	data->vortex_ex_adc_hwmon =
		hwmon_device_register_with_groups(&pdev->dev, DRV_NAME,
						  pci_get_drvdata(pdev),
						  analog_in_groups);
	if (IS_ERR(data->vortex_ex_adc_hwmon)) {
		err = PTR_ERR(data->vortex_ex_adc_hwmon);
		goto failure_sysfs;
	}

	return 0;

failure_sysfs:
	hwmon_device_unregister(data->vortex_ex_adc_hwmon);
failure_request_mem_region:
	release_region(VORTEX_ADC_PHYS_ADDR, VORTEX_ADC_LEN);
failure_disable_pci:
	dev_dbg(&pdev->dev, "Error path : %d", err);

	/* Disable ADC */
	pci_readmodifywrite_config_dword(pdev,
					 (DEV_SB << 3) | VORTEX_EX_SB_FN0,
					 FN0_REG_ENABLE,
					 (1L << ENABLE_ADC_BIT),
					 (1L << ENABLE_ADC_BIT));

	pci_disable_device(pdev);
	return err;
}

static void vortex_adc_remove(struct pci_dev *pdev)
{
	/* Clean up any allocated resources and stuff here.
	 * like call release_region();
	 */
	struct vortex_adc_data *data = pci_get_drvdata(pdev);

	hwmon_device_unregister(data->vortex_ex_adc_hwmon);

	/* Disable ADC */
	pci_readmodifywrite_config_dword(pdev,
					 (DEV_SB << 3) | VORTEX_EX_SB_FN0,
					 FN0_REG_ENABLE,
					 (1L << ENABLE_ADC_BIT),
					 (1L << ENABLE_ADC_BIT));


	release_region(VORTEX_ADC_PHYS_ADDR, VORTEX_ADC_LEN);
	data->base_address = 0;
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static struct pci_driver vortex_adc_driver = {
	.name = DRV_NAME,
	.id_table = vortex_ex_pci_tbl,
	.probe = vortex_adc_probe,
	.remove = vortex_adc_remove,
};

/*
static int __init vortexEx_adc_init(void)
{
	return pci_register_driver( &vortex_adc_driver);
}

static void __exit vortexEx_adc_exit(void)
{
	pci_unregister_driver(&vortex_adc_driver);
}


module_init(vortexEx_adc_init);
module_exit(vortexEx_adc_exit);
*/
module_pci_driver(vortex_adc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DMP Vortex Ex ADC");
MODULE_AUTHOR("Jan.Cornelis@gmail.com");
