#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/completion.h>

#include <linux/mtd/mtd.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/completion.h>
#include <linux/hardirq.h>
#include <linux/irqflags.h>
#include <linux/rwsem.h>

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/switch.h>
#include <linux/interrupt.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include <linux/rtpm_prio.h>
#include <linux/sched.h>

static struct pinctrl *pinctrl_hall;
static struct pinctrl_state *pins_hall_eint_default;
static struct pinctrl_state *pin_hall_eint_as_int;

static unsigned int hall_irq;
static unsigned int gpiopin, halldebounce;
static unsigned int hall_eint_type = IRQ_TYPE_LEVEL_LOW;

#define EINT_PIN_FAR        (1)
#define EINT_PIN_CLOSE       (0)
int cur_hall_eint_state = EINT_PIN_FAR;

static struct task_struct *thread = NULL;
static int hall_eint_flag = 0;
static unsigned int hall_eint_close = 0;



static const struct of_device_id mt_hall_eint_of_match[] = {
	{.compatible = "mediatek,mthalleint"},
	{},
};


//////////////////////////////////////////////
static struct input_dev *hall_eint_idev;

//////////////////////////////////////////////

//////////////////////////////////////////////
static struct switch_dev mthalleint;
static void hall_enit_init(void)
{
	 mthalleint.name = "mthalleint";
	 mthalleint.index = 0;
	 mthalleint.state = 1;
 
	 if(switch_dev_register(&mthalleint))
		 printk("switch_dev_register mthalleint fail\n");
	 else
		 printk("switch_dev register mthalleint success\n");
	 
	 switch_set_state((struct switch_dev *)&mthalleint, 1);

}
/////////////////////////////////////////////////

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static irqreturn_t hall_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	printk("hall_eint_interrupt_handler");

	hall_eint_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	disable_irq_nosync(hall_irq);
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int hall_eint_level=IRQ_TYPE_LEVEL_LOW;
static int hall_eint_event_handler(void *unused)
{

	printk("hall_eint_event_handler interrupt hall_eint_level=%d\n",hall_eint_level);
	do{
			
			set_current_state(TASK_INTERRUPTIBLE); 
			wait_event_interruptible(waiter,hall_eint_flag!=0);
			hall_eint_flag = 0;
			set_current_state(TASK_RUNNING);
		if (hall_eint_level==IRQ_TYPE_LEVEL_LOW)
		{	
			printk("hall_eint_event_handler interrupt hall_eint_level=%d\n",hall_eint_level);

			input_report_key(hall_eint_idev, KEY_F11, 1);//KEY_MACRO
			input_sync(hall_eint_idev);
			msleep(2);
			input_report_key(hall_eint_idev, KEY_F11, 0);
			input_sync(hall_eint_idev);

			hall_eint_close=1;//close
		}
		else
		{
			printk("hall_eint_event_handler interrupt hall_eint_level=%d\n",hall_eint_level);

			input_report_key(hall_eint_idev, KEY_F12, 1);//KEY_MACRO
			input_sync(hall_eint_idev);
			msleep(2);
			input_report_key(hall_eint_idev, KEY_F12, 0);
			input_sync(hall_eint_idev);

			hall_eint_close=0;//far away
		}
		switch_set_state((struct switch_dev *)&mthalleint, hall_eint_close);
		if(hall_eint_level==IRQ_TYPE_LEVEL_LOW)
			{
			irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_HIGH);
		    hall_eint_level=IRQ_TYPE_LEVEL_HIGH;
			}
		else
			{
			irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_LOW);
		    hall_eint_level=IRQ_TYPE_LEVEL_LOW;
			}
		enable_irq(hall_irq);
	}while(!kthread_should_stop());
		
	return 0;
}

static int hall_eint_probe(struct platform_device *pdev)
{	
	int ret;
	struct device_node *node = NULL;
	u32 ints[2] = {0,0};
	u32 ints1[2] = { 0, 0 };

	printk("hall_eint_probe------\n");

	
  //gpio init start
	pinctrl_hall = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl_hall)) {
		ret = PTR_ERR(pinctrl_hall);
		printk("fwq Cannot find hall eint pinctrl_hall!\n");
		//return ret;
	}
	pins_hall_eint_default = pinctrl_lookup_state(pinctrl_hall, "hall_eint_endefault");
	//if (IS_ERR(pins_hall_eint_default)) {
	//	ret = PTR_ERR(pins_hall_eint_default);
	//	printk("fwq Cannot find hall eint pinctrl hall_eint_endefault %d!\n", ret);
	//}
	pin_hall_eint_as_int = pinctrl_lookup_state(pinctrl_hall, "hall_eint_as_int");
	if (IS_ERR(pin_hall_eint_as_int)) {
		ret = PTR_ERR(pin_hall_eint_as_int);
		printk("fwq Cannot find hall eint pinctrl hall_eint_as_int!\n");
		//return ret;
	}
	//gpio init end

	//eint registration start
	node = of_find_matching_node(node, mt_hall_eint_of_match);
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		gpiopin = ints[0];
		halldebounce = ints[1];
		hall_eint_type = ints1[1];

		gpio_set_debounce(gpiopin, halldebounce);
		
		hall_irq = irq_of_parse_and_map(node, 0);
		printk("Device hall_irq = %d!", hall_irq);
		
		ret = request_irq(hall_irq, (irq_handler_t)hall_eint_interrupt_handler, IRQF_TRIGGER_HIGH,"mthalleint", NULL);
		if(ret > 0){
			ret = -1;
			return ret;
			printk("hall_irq request_irq IRQ LINE NOT AVAILABLE!.");
		}
    }
	//eint registration start

	thread = kthread_run(hall_eint_event_handler, 0, "mthalleint");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		printk("hall_eint_probe failed to create kernel thread: %d\n", ret);
	}


	/*******hall_eint_idev add input device start*******/
	hall_eint_idev = input_allocate_device();
	if (!hall_eint_idev) 
	{
		printk("alloc input device failed!\n");
		// ret = -ENOMEM;
	}
	hall_eint_idev->name = "mthalleint";
	hall_eint_idev->id.bustype = BUS_HOST;//;
	hall_eint_idev->dev.parent = &pdev->dev;;

	input_set_capability(hall_eint_idev,EV_KEY,KEY_F11);//
	input_set_capability(hall_eint_idev,EV_KEY,KEY_F12);//
	
	__set_bit(EV_KEY, hall_eint_idev->evbit);
	ret = input_register_device(hall_eint_idev);
	if (ret) 
	{
		printk("register device failed!\n");
	}
	/*******hall_eint_idev add input device end*******/
	
	printk("hall_eint_probe-ok-----\n");
	return 0;
}




static int hall_eint_remove(struct platform_device *pdev)
{

	input_unregister_device(hall_eint_idev);
	input_free_device(hall_eint_idev);

	return 0;
}


#if 0
struct platform_device hall_eint_device = 
{
	.name	 = "mthalleint",
	//.id	 = -1,
};
#endif

static struct platform_driver hall_eint_driver = {
	.driver	  = {
	.name   = "mthalleint",
	.owner = THIS_MODULE,
     .of_match_table = mt_hall_eint_of_match,
	},
	.probe	  = hall_eint_probe,
	.remove	  = hall_eint_remove,
};

/* called when loaded into kernel */
static int __init hall_eint_driver_init(void) 
{
	printk("MediaTek hall_eint driver init\n");
	//if(platform_device_register(&hall_eint_device))
	//{
	//	printk("MediaTek hall_eint_device add err\n");
	//	return -2;
	//}

	if(platform_driver_register(&hall_eint_driver) < 0)
	{
		printk("add hall_eint driver failed\n");
		return -1;
	}

	hall_enit_init();
	
	return 0;
}
 
/* should never be called */
static void __exit hall_eint_driver_exit(void) 
{
	printk("MediaTek hall_eint exit\n");

	platform_driver_unregister(&hall_eint_driver);

}
 

module_init(hall_eint_driver_init);
module_exit(hall_eint_driver_exit);


/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("sunmontech jackymao");
MODULE_DESCRIPTION("hall_eint driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("hall_eint");


