#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define IMX6UIRQ_CNT   1    //设备号个数
#define IMX6UIRQ_NAME  "asyncnoti"  //名字
#define KEY0VALUE      0x01     //KEY0按键对应的键值
#define INVAKEY        0xef    
#define KEY_NUM        1


//中断IO描述结构体
struct irq_keydesc{
    int gpio; 
    int irqnum;   //中断号
    unsigned char value;  //按键对应的键值
    char name[10];
    irqreturn_t (*handler)(int, void *);   //中断服务函数
};

//imx6uirq设备结构体
struct imx6uirq_dev{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    int major;
    int minor;
    struct device_node *nd;
    atomic_t keyvalue;    //有效的按键键值
    atomic_t releasekey;   //标记是否完成一次的按键
    struct timer_list timer;
    struct irq_keydesc irqkeydesc[KEY_NUM];  //按键描述数组
    unsigned char curkeynum;    //当前的按键号

    wait_queue_head_t r_wait;  //读等待队列头
    struct fasync_struct *async_queue;
};

struct imx6uirq_dev imx6uirq;   //irq设备

static int key_flag = 0;

//中断服务函数,按下按键和释放按键都会触发这里
//注意: 抖动是由于按下按键或者释放按键才引起的，不操作按键时不会引发抖动！
static irqreturn_t key0_handler(int irq, void *dev_id){
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)dev_id;

    dev->curkeynum = 0;
    dev->timer.data = (volatile long)dev_id;
    //按键按下时，设置超时时间，并且这个函数会启动定时器
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10)); //10ms
    return IRQ_RETVAL(IRQ_HANDLED);
}

/*定时器服务函数，用于按键消抖，定时器到了以后再次读取按键值，如果按键还是处于按下
状态就表示按键有效 */
//这个函数执行时，说明距离按键按下已经过去10ms了。
void timer_function(unsigned long arg){
    unsigned char value;
    unsigned char num;
    struct irq_keydesc *keydesc;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)arg;

    num = dev->curkeynum;
    keydesc = &dev->irqkeydesc[num];

    value = gpio_get_value(keydesc->gpio);   //读取IO值
    if(value == 0){  //当前状态是按下按键(需要等到松开按键时才表示一次完整的按键操作！）
        atomic_set(&dev->keyvalue, keydesc->value);
    }
    else{   //按键松开
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);  //标记松开按键
    }

    if(atomic_read(&dev->releasekey)){
    /*
      当设备可以访问的时候，驱动程序需要向应用程序发出信号，相当于产生"中断"。 
      kill_fasync函数负责发送指定的信号
    */
        if(dev->async_queue)
            kill_fasync(&dev->async_queue, SIGIO, POLL_IN);   //band：可读时设置为 POLL_IN，可写时设置为 POLL_OUT
    }

}

//fasync函数，用于处理异步通知
/**
 * on:模式
*/
/*
    当应用程序通过fcntl(fd, F_SETFL, flags | FASYNC)改变
    fasync标记的时候，驱动程序file_operations操作集中的fasync函数就会执行。
*/
static int imx6uirq_fasync(int fd, struct file *filp, int on){
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)filp->private_data;
    return fasync_helper(fd, filp, on, &dev->async_queue);  //该函数用于初始化fasync_struct结构体
}

static int imx6uirq_release(struct inode *inode, struct file *filp){
    return imx6uirq_fasync(-1, filp, 0);
}

//按键IO初始化
static int keyio_init(void){
    unsigned char i = 0;
    int ret = 0;
    imx6uirq.nd = of_find_node_by_path("/key");
    if(imx6uirq.nd == NULL){
        printk("key node not find!\r\n");
        return -EINVAL;
    }

    //提取GPIO
    for(i = 0; i < KEY_NUM; i++){
        imx6uirq.irqkeydesc[i].gpio = of_get_named_gpio(imx6uirq.nd, "key-gpio", i);
        if(imx6uirq.irqkeydesc[i].gpio < 0){
            printk("can't get key%d\r\n", i);
        }
    }

    //初始化Key所使用的IO，并设置成中断模式
    for(i = 0; i < KEY_NUM; i++){
        memset(imx6uirq.irqkeydesc[i].name, 0, sizeof(imx6uirq.irqkeydesc[i].name));
        sprintf(imx6uirq.irqkeydesc[i].name, "KEY%d", i);
        gpio_request(imx6uirq.irqkeydesc[i].gpio, imx6uirq.irqkeydesc[i].name);
        gpio_direction_input(imx6uirq.irqkeydesc[i].gpio);
        imx6uirq.irqkeydesc[i].irqnum = irq_of_parse_and_map(imx6uirq.nd, i); //中断号

#if 0
        imx6uirq.irqkeydesc[i].irqnum = gpio_to_irq(imx6uirq.irqkeydesc[i].gpio);
#endif  
        printk("key%d:gpio=%d, irqnum=%d\r\n", i, imx6uirq.irqkeydesc[i].gpio,
                                            imx6uirq.irqkeydesc[i].irqnum);
    }
    //申请中断
    imx6uirq.irqkeydesc[0].handler = key0_handler;
    imx6uirq.irqkeydesc[0].value = KEY0VALUE;

    for(i = 0; i < KEY_NUM; i++){
        ret = request_irq(imx6uirq.irqkeydesc[i].irqnum,
                          imx6uirq.irqkeydesc[i].handler,
                          IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                          imx6uirq.irqkeydesc[i].name, &imx6uirq);
        if(ret < 0){
            printk("irq %d request failed!\r\n", imx6uirq.irqkeydesc[i].irqnum);
            return -EFAULT;
        }
    }

    //创建定时器
    init_timer(&imx6uirq.timer);
    imx6uirq.timer.function = timer_function;

    //初始化等待队列头
    init_waitqueue_head(&imx6uirq.r_wait);
    return 0;
}

//打开设备
static int imx6uirq_open(struct inode *inodoe, struct file *filp){
    filp->private_data = &imx6uirq;   
    return 0;
}

//从设备读数据
static ssize_t imx6uirq_read(struct file *filp, char __user *buf,
                            size_t cnt, loff_t *offt)
{
    int ret = 0;
    unsigned char keyvalue = 0;
    unsigned char releasekey = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)filp->private_data;

    if(filp->f_flags & O_NONBLOCK){
        if(atomic_read(&dev->releasekey) == 0)
            return -EAGAIN;
    } else{
        /**
         * 在wait_event_interruptible()函数中会将当前进程的状态设置成TASK_INTERRUPTIBLE，
         * 然后调用schedule()，它会将位于TASK_INTERRUPTIBLE状态的进程从run queue队列中删除
         * (之前阻塞实验中貌似是手动实现了一下这个过程。是的！就是这样)
        */
        // atomic_read(&dev->releasekey)表示读取&dev->releasekey的值，并且返回
        // 当&dev->releasekey的值为1时才表示按键完整按下一次。从等待队列中唤醒该进程
        ret = wait_event_interruptible(dev->r_wait, atomic_read(&dev->releasekey));
        if(ret){
            goto wait_error;
        }
    }

    keyvalue = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);
    if(releasekey){
        if(keyvalue & 0x80){
            keyvalue &= ~0x80;
            ret = copy_to_user(buf, &keyvalue, sizeof(keyvalue));
        } else{
            goto data_error;
        }
        atomic_set(&dev->releasekey, 0);  //按下标志清零
    } else{
        goto data_error;
    }
    return 0;

wait_error:
    return ret;

data_error:
    return -EINVAL;
}

//该函数是非阻塞IO实验中用到，当前文件中其实没用到
unsigned int imx6uirq_poll(struct file *filp, struct poll_table_struct *wait){
    unsigned int mask = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)filp->private_data;
    poll_wait(filp, &dev->r_wait, wait); //将等待队列头添加到poll_talbe中
    if(atomic_read(&dev->releasekey)){
        mask = POLLIN | POLLRDNORM;
    }
    return mask;
}

//设备操作函数
static struct file_operations imx6uirq_fops = {
    .owner = THIS_MODULE,
    .open = imx6uirq_open,
    .read = imx6uirq_read,
    .poll = imx6uirq_poll,
    .fasync = imx6uirq_fasync,
    .release = imx6uirq_release,
};

//驱动入口函数
static int __init imx6uirq_init(void){
    //1、构建设备号
    if(imx6uirq.major){
        imx6uirq.devid = MKDEV(imx6uirq.major, 0);
        register_chrdev_region(imx6uirq.devid, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    } else{
        alloc_chrdev_region(&imx6uirq.devid, 0, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
        imx6uirq.major = MAJOR(imx6uirq.devid);
        imx6uirq.minor = MINOR(imx6uirq.devid);
    }

    //2、注册字符设备
    cdev_init(&imx6uirq.cdev, &imx6uirq_fops);
    cdev_add(&imx6uirq.cdev, imx6uirq.devid, IMX6UIRQ_CNT);

    //3、创建类
    imx6uirq.class = class_create(THIS_MODULE, IMX6UIRQ_NAME);
    if(IS_ERR(imx6uirq.class)){
        return PTR_ERR(imx6uirq.class);
    }

    //4、创建设备
    imx6uirq.device = device_create(imx6uirq.class, NULL, imx6uirq.devid, NULL, IMX6UIRQ_NAME);

    if(IS_ERR(imx6uirq.device)){
        return PTR_ERR(imx6uirq.device);
    }

    //5、初始化按键
    atomic_set(&imx6uirq.keyvalue, INVAKEY);
    atomic_set(&imx6uirq.releasekey, 0);
    keyio_init();
    return 0;
}

//驱动出口函数
static void __exit imx6uirq_exit(void){
    unsigned int i = 0;
    //删除定时器
    del_timer_sync(&imx6uirq.timer);

    //释放中断
    for(i = 0; i < KEY_NUM; i++){
        free_irq(imx6uirq.irqkeydesc[i].irqnum, &imx6uirq);
    }

    cdev_del(&imx6uirq.cdev);
    unregister_chrdev_region(imx6uirq.devid, IMX6UIRQ_CNT);
    device_destroy(imx6uirq.class, imx6uirq.devid);
    class_destroy(imx6uirq.class);
}

module_init(imx6uirq_init);
module_exit(imx6uirq_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("rsh");
