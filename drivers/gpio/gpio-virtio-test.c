#include <linux/gpio.h>
#include <linux/interrupt.h>

static struct completion completion;
static irqreturn_t __maybe_unused TEST_irq_handler_empty(int irq, void *pw)
{
	pr_info("%s: %d: %d\n", __func__, __LINE__, irq);
	complete(&completion);
	return IRQ_HANDLED;
}

static irqreturn_t __maybe_unused TEST_irq_handler_disable(int irq, void *pw)
{
	pr_info("%s: %d: %d\n", __func__, __LINE__, irq);
	complete(&completion);
	disable_irq(irq);
	return IRQ_HANDLED;
}

static void __maybe_unused TEST_my_request_irq(int gpio, void *handler, int flag)
{
	int ret;

	pr_info("%s: %d: %d\n", __func__, __LINE__, gpio);
	ret = gpio_request_one(gpio, GPIOF_DIR_IN, "test");
	if (ret) {
		pr_err("failed to request IRQ GPIO: %d\n", ret);
		return;
	}

	ret = request_irq(gpio_to_irq(gpio), handler, flag, __func__, NULL);
	if (ret != 0)
		pr_err("failed to request ...: %d\n", ret);
}

static void __maybe_unused TEST_my_request_irq_threaded(int gpio, void *handler, int flag)
{
	int ret;

	pr_info("%s: %d: %d\n", __func__, __LINE__, gpio);
	ret = gpio_request_one(gpio, GPIOF_DIR_IN, "test");
	if (ret) {
		pr_err("failed to request IRQ GPIO: %d\n", ret);
		return;
	}

	ret = request_threaded_irq(gpio_to_irq(gpio), NULL, handler, flag, __func__, NULL);
	if (ret != 0)
		pr_err("failed to request ...: %d\n", ret);
}

struct test_configuration {
	void (*register_irq)(int gpio, void *handler, int flag);
	irqreturn_t (*handler)(int irq, void *pw);
	unsigned int flags;
};

static struct test_configuration tests[] = {
	{ TEST_my_request_irq, TEST_irq_handler_empty, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING },
	{ TEST_my_request_irq, TEST_irq_handler_empty, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT},
//	{ TEST_my_request_irq, TEST_irq_handler_disable, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING },
//	{ TEST_my_request_irq, TEST_irq_handler_disable, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT},
	{ TEST_my_request_irq_threaded, TEST_irq_handler_empty, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT},
	{ TEST_my_request_irq_threaded, TEST_irq_handler_disable, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT},

//	{ TEST_my_request_irq, TEST_irq_handler_empty, IRQF_TRIGGER_LOW },
//	{ TEST_my_request_irq, TEST_irq_handler_empty, IRQF_TRIGGER_LOW | IRQF_ONESHOT},
////	{ TEST_my_request_irq, TEST_irq_handler_disable, IRQF_TRIGGER_LOW },
////	{ TEST_my_request_irq, TEST_irq_handler_disable, IRQF_TRIGGER_LOW | IRQF_ONESHOT},
//	{ TEST_my_request_irq_threaded, TEST_irq_handler_empty, IRQF_TRIGGER_LOW | IRQF_ONESHOT},
//	{ TEST_my_request_irq_threaded, TEST_irq_handler_disable, IRQF_TRIGGER_LOW | IRQF_ONESHOT},
};

static void test_gpio(int base)
{
	struct test_configuration *test;
	int i;

	init_completion(&completion);

	for (i = 0; i < ARRAY_SIZE(tests); i++) {
		test = tests + i;
		test->register_irq(base + i, test->handler, test->flags);
		wait_for_completion(&completion);
		pr_info("\n\n\n");
	}

	for (i = 0; i < ARRAY_SIZE(tests); i++) {
		test = tests + i;

		if (test->handler == TEST_irq_handler_disable)
			enable_irq(gpio_to_irq(base + i));
	}

	disable_irq(gpio_to_irq(base));
}
