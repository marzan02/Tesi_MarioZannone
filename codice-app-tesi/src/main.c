#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

/* Verifica alias dall'overlay */
#define OK(a) DT_NODE_HAS_STATUS(DT_ALIAS(a), okay)
BUILD_ASSERT(OK(led0) && OK(led1) && OK(led2) && OK(led3) && OK(sw0),
             "Mancano alias led0/1/2/3 e/o sw0");

static const struct gpio_dt_spec LED0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); /* verde */
static const struct gpio_dt_spec LED1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); /* verde */
static const struct gpio_dt_spec LEDR = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios); /* rosso */
static const struct gpio_dt_spec LEDB = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios); /* blu   */
static const struct gpio_dt_spec BTN  = GPIO_DT_SPEC_GET(DT_ALIAS(sw0),  gpios); /* PA0   */

/* Stato: 1 = lampeggia ROSSO, 0 = lampeggia BLU */
static atomic_t red_enabled  = ATOMIC_INIT(1);
static atomic_t blue_enabled = ATOMIC_INIT(0);

/* Lettura pulsante (active-high) */
static inline int btn_read(void) { return gpio_pin_get_dt(&BTN) > 0; }


static void blink_thread(void *p_gpio, void *p_period_ms, void *p_enable_atomic)
{
    const struct gpio_dt_spec *led = (const struct gpio_dt_spec *)p_gpio;
    uint32_t period_ms = (uint32_t)(uintptr_t)p_period_ms;
    atomic_t *en = (atomic_t *)p_enable_atomic;

    while (1) {
        if (!en || atomic_get(en)) {
            gpio_pin_toggle_dt(led);
            k_msleep(period_ms / 2);
            gpio_pin_toggle_dt(led);
            k_msleep(period_ms / 2);
        } else {
            gpio_pin_set_dt(led, 0);
            k_msleep(50);
        }
    }
}

/* Stack & TCB LED */
K_THREAD_STACK_DEFINE(stk_l0, 1024);
K_THREAD_STACK_DEFINE(stk_l1, 1024);
K_THREAD_STACK_DEFINE(stk_r,  1024);
K_THREAD_STACK_DEFINE(stk_b,  1024);
static struct k_thread th_l0, th_l1, th_r, th_b;

//test con thread lento
static void test(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* Facciamo piccoli tratti di busy-wait per non bloccare le IRQ
       e dare occasione al scheduler di preemptare */
    while (1) {
        k_busy_wait(7000);   
        k_msleep(1);         
    }
}
K_THREAD_STACK_DEFINE(stk_lento, 1024);
static struct k_thread th_lento;

/* === IRQ pulsante: ISR minima + work item === */
#define DEBOUNCE_MS 80
static volatile int64_t last_irq_ms;
static struct gpio_callback btn_cb;

/* Work eseguito fuori dalla ISR */
static void on_button_press(struct k_work *work);
K_WORK_DEFINE(btn_work, on_button_press);

static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);

    int64_t now = k_uptime_get();
    if ((now - last_irq_ms) < DEBOUNCE_MS) {
        return; /* debounce temporale in ISR, niente sleep */
    }
    last_irq_ms = now;

    /* extra check di livello (active-high) */
    if (!btn_read()) {
        return;
    }
    //test ISR più pesante
    //k_busy_wait(500000);

    k_work_submit(&btn_work);
}

static void quick_double_flash_led1(void)
{
    gpio_pin_set_dt(&LED1, 1); k_msleep(60);
    gpio_pin_set_dt(&LED1, 0); k_msleep(60);
    gpio_pin_set_dt(&LED1, 1); k_msleep(60);
    gpio_pin_set_dt(&LED1, 0);
}

static void on_button_press(struct k_work *work)
{
    /* Toggle ROSSO↔BLU */
    int now_red = !atomic_get(&red_enabled);
    atomic_set(&red_enabled,  now_red);
    atomic_set(&blue_enabled, !now_red);

    /* Feedback visivo chiaro */
    quick_double_flash_led1();
}

int main(void)
{
    if (!device_is_ready(LED0.port) || !device_is_ready(LED1.port) ||
        !device_is_ready(LEDR.port) || !device_is_ready(LEDB.port) ||
        !device_is_ready(BTN.port)) {
        return 0;
    }

    /* LED output */
    gpio_pin_configure_dt(&LED0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&LED1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&LEDR, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&LEDB, GPIO_OUTPUT_INACTIVE);

    /* Pulsante: modalità NO_PULL */
    gpio_pin_configure_dt(&BTN, GPIO_INPUT);

    /* IRQ su fronte attivo (rising, perché active-high) */
    gpio_pin_interrupt_configure_dt(&BTN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb, btn_isr, BIT(BTN.pin));
    gpio_add_callback(BTN.port, &btn_cb);

    /* Avvio: verdi fissi, rosso/blu esclusivi (parte ROSSO) */
    atomic_set(&red_enabled, 1);
    atomic_set(&blue_enabled, 0);

    k_thread_create(&th_l0, stk_l0, K_THREAD_STACK_SIZEOF(stk_l0),
                    blink_thread, (void*)&LED0, (void*)(uintptr_t)2000, NULL,
                    K_PRIO_PREEMPT(8), 0, K_NO_WAIT);

    k_thread_create(&th_l1, stk_l1, K_THREAD_STACK_SIZEOF(stk_l1),
                    blink_thread, (void*)&LED1, (void*)(uintptr_t)4000, NULL,
                    K_PRIO_PREEMPT(8), 0, K_NO_WAIT);

    k_thread_create(&th_r, stk_r, K_THREAD_STACK_SIZEOF(stk_r),
                    blink_thread, (void*)&LEDR, (void*)(uintptr_t)1000, &red_enabled,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

    k_thread_create(&th_b, stk_b, K_THREAD_STACK_SIZEOF(stk_b),
                    blink_thread, (void*)&LEDB, (void*)(uintptr_t)1000, &blue_enabled,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

    //thread lento
    k_thread_create(&th_lento, stk_lento, K_THREAD_STACK_SIZEOF(stk_lento),
                    test, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(9), 0, K_NO_WAIT);

    // k_thread_create(&th_lento, stk_lento, K_THREAD_STACK_SIZEOF(stk_lento),
    //                 test, NULL, NULL, NULL,
    //                 K_PRIO_PREEMPT(4), 0, K_NO_WAIT);

    /* Loop inattivo (tutto avviene nei thread e nell'IRQ) */
    while (1) {
        k_msleep(1000);
    }
    return 0;
}
