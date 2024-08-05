#include <linux/module.h>       //所有模块都需要的头文件
#include <linux/init.h>         // init和exit相关宏
#include <linux/kernel.h>       // printk()，内核打印函数
#include <linux/delay.h>        // 延时函数头文件
#include <linux/device.h>       // 用于设备创建的函数头文件
#include <linux/fs.h>           //和fops相关的头文件
#include <linux/cdev.h>         //字符设备 初始化相关
#include <linux/version.h>
//#include <linux/ide.h>
#include <linux/gpio.h>         //gpio子系统头文件
#include <linux/of_gpio.h>      //gpio子系统和设备树相关
#include <linux/platform_device.h>  //platform总线设备相关
#include <linux/err.h>          //错误码相关
#include <linux/timer.h>        //定时器相关
#include <linux/i2c.h>          //i2c子系统相关
#include <linux/uaccess.h>

#define DEV_NAME                        "sht20"  //设备名
#define SHT20_SOFERESET                 0xFE    // 软复位
#define SHT20_TEMPERATURE_NO_HOLD_CMD   0xF3    // 无主机模式触发温度测量
#define SHT20_HUMIDITY_NO_HOLD_CMD      0xF5    // 无主机模式触发湿度测量
#define SHT20_TEMPERATURE_HOLD_CMD      0xE3    // 主机模式触发温度测量
#define SHT20_HUMIDITY_HOLD_CMD         0xE5    // 主机模式触发湿度测量

#define CRC_MODEL                   0x131
#define CRC_SUCCESS                 0
#define CRC_FAIL                    1

#ifndef DEV_MAJOR
#define DEV_MAJOR       0
#endif

static int dev_major = DEV_MAJOR;   //主机号

struct sht20_priv {
	struct cdev         cdev;
	struct class        *dev_class;
	struct i2c_client   *client;
	struct device       *dev;
};

/* @description: 初始化sht20
 *
 * @parm  : client - i2c 设备
 * @parm  : 
 * @return: 0 successfully , !0 failure
 */
static int sht20_init(struct i2c_client *client)
{    
	int rv;
	char data = SHT20_SOFERESET;

	rv = i2c_master_send(client, &data, 1);
	if(rv < 0)
	{
		dev_err(&client->dev, "i2c send init cmd failure.\n");
		return -1;
	}

	msleep(50);
	return 0;
}

static int crc_check(unsigned char *data, int len, unsigned char checksum)
{
	unsigned char   crc = 0x00; 
	int             i, j;  

	for(i=0; i<len; i++)
	{
		crc ^= *data++;  		   

		for (j=0; j<8; j++)     
		{ 
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ CRC_MODEL;
			}    
			else
			{
				crc = (crc << 1);
			}
		}
	}
	// printk("crc clu data : [%x]\n", crc);

	if(checksum == crc)
	{
		return CRC_SUCCESS;
	}
	else 
	{
		return CRC_FAIL;
	}
}

/* @description: 读取sht20 的温度数据 
 *
 * @parm  : client - i2c 设备
 * @parm  : buf - 存储读取的数据
 * @return: 0 successfully , !0 failure
 */
static int read_temperature(struct i2c_client *client, unsigned char *buf)
{
	int rv = 0;
	int temperature = 0;

	unsigned char tmp[3] = {0};
	char data = SHT20_TEMPERATURE_HOLD_CMD;

	//形参判断
	if(!client || !buf)
	{
		printk("%s line [%d] %s() get invalid input arguments\n", __FILE__, __LINE__, __func__ );
		return -1;
	}

	//发送CMD
	rv = i2c_master_send(client, &data, 1);
	//rv = i2c_smbus_write_byte(client, SHT20_TEMPERATURE_NO_HOLD_CMD);
	if(rv < 0)
	{
		dev_err(&client->dev, "i2c send tmper cmd failure.\n");
		return -1;
	}

	//delay 85ms
	msleep(85);

	//读取数据
	rv = i2c_master_recv(client, tmp, sizeof(tmp));
	if(rv < 0)
	{
		dev_err(&client->dev, "i2c recv tmper data failure.\n");
		return -1; 
	}

	// printk("read temperature: tmp[0] %x tmp[1] %x ; crc : tmp[2] %x\n", tmp[0], tmp[1], tmp[2]); //验证 crc校验结果
	//数据处理
	temperature = (tmp[0] << 8) | (tmp[1]&0xFC);
	temperature = ((temperature * 175720) >> 16) - 46850;

	//printk("temperature : %d\n", temperature);
	//TODO: 可以加上CRC校验
	if(0 != crc_check(tmp, 2, tmp[2]))
	{
		dev_err(&client->dev, "tmperature data fails to pass cyclic redundancy check\n");
		return -1;
	}

	buf[0] = temperature & 0xFF;
	buf[1] = (temperature >> 8) & 0xFF;

	return 0;
}

/* @description: 读取sht20 的相对数据数据
 *
 * @parm  : client - i2c 设备/客户端
 * @parm  : buf - 存储读取的数据
 * @return: 0 successfully , !0 failure
 */
static int read_humidity(struct i2c_client *client, unsigned char *buf)
{
	int rv = 0;
	int humidity = 0;

	unsigned char tmp[3] = {0};
	char data = SHT20_HUMIDITY_HOLD_CMD;

	//形参判断
	if(!client || !buf)
	{
		printk("%s line [%d] %s() get invalid input arguments\n", __FILE__, __LINE__, __func__ );
		return -1;
	}

	//发送CMD
	rv = i2c_master_send(client, &data, 1);
	//rv = i2c_smbus_write_byte(client, SHT20_HUMIDITY_NO_HOLD_CMD);
	if(rv < 0)
	{
		dev_err(&client->dev, "i2c send humidity cmd failure.\n");
		return -1;
	}

	//delay 29ms
	msleep(29);

	//读取数据
	rv = i2c_master_recv(client, tmp, sizeof(tmp));
	if(rv < 0)
	{
		dev_err(&client->dev, "i2c recv humidity data failure.\n");
		return -1; 
	}

	// printk("read humidity: tmp[0] %x tmp[1] %x ; crc : tmp[2] %x\n", tmp[0], tmp[1], tmp[2]);

	//数据处理
	humidity = (tmp[0] << 8) | (tmp[1]&0xFC);
	humidity = ((humidity * 125000) >> 16) - 6000;

	//crc-8 校验
	if(0 != crc_check(tmp, 2, tmp[2]))
	{
		dev_err(&client->dev, "tmperature data fails to pass cyclic redundancy check\n");
		return -1;
	}

	buf[0] = humidity & 0xFF;
	buf[1] = (humidity >> 8) & 0xFF;

	return 0;
}


/* @description: sht20 打开设备 
 *
 * @parm  : inode - 传递给驱动的inode
 * @parm  : filp - 设备文件，利用其私有数据成员
 * @return: 0 successfully , !0 failure
 */
static int sht20_open(struct inode *inode, struct file *filp)
{
	struct sht20_priv *priv = container_of(inode->i_cdev, struct sht20_priv, cdev);  
	int rv;

	//初始化sht20 
	rv = sht20_init(priv->client);
	if(rv < 0)
	{
		dev_err(priv->dev, "sht20 init failure.\n");
	}
	//dev_info(priv->dev, "sht20 init successfully.\n");

	filp->private_data = priv;

	return 0;
}

/* @description: 从设备读取文件
 *
 * @parm  : filp - 设备文件，文件描述符
 * @parm  : buf - 返回给用户空间的数据缓冲区
 * @parm  : cnt - 要读取的数据长度
 * @parm  : offt - 相对于文件首地址的偏移
 * @return: 读取的字节数，负数 - 读取失败
 */
static ssize_t sht20_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int rv = 0;
	//struct i2c_client *client = filp->private_data;
	struct sht20_priv *priv = filp->private_data;
	unsigned char data[4] = {0,};

	if(!priv->client)
	{
		printk("failure to get i2c_client.\n");
		return -EFAULT;
	}

	rv = read_temperature(priv->client, data);
	if(rv)
	{
		dev_err(priv->dev, "read_temperature failure.\n");
	}

	data[3] = data[1];
	data[2] = data[0];

	rv = read_humidity(priv->client, data);
	if(rv)
	{
		dev_err(priv->dev, "read_humidity failure.\n");
	}

	//printk("test %x %x %x %X \n", data[3], data[2], data[1], data[0]);
	rv = copy_to_user(buf, data, sizeof(data));
	if(rv)
	{
		dev_err(priv->dev, "copy to user error.\n");
		return -EFAULT;
	}

	return sizeof(data);
}

/* @description: 关闭设备
 *
 * @parm  : inode - 传递给驱动的inode
 * @parm  : filp - 设备文件，file结构体有个私有数据区可以使用
 * @return: 0 successfully , !0 failure
 */
static int sht20_release(struct inode *inode, struct file *filp)
{
	return 0;
}

//设备操作函数
static struct file_operations sht20_fops = {
	.owner = THIS_MODULE,
	.open  = sht20_open,
	.read  = sht20_read,
	.release = sht20_release,
};

/* @description: sysfs - 温度属性显示函数
 *
 * @parm  : dev - 设备指针,创建file时候会指定dev
 * @parm  : attr - 设备属性,创建时候传入
 * @parm  : buf - 传出给sysfs中显示的buf
 * @return: 显示的字节数
 * @TODO: 函数不够正规,了解PAGE_SIZE
 */
static ssize_t sht20_temp_humi_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sht20_priv *priv = dev_get_drvdata(dev);
	int rv = 0;
	unsigned char data[4] = {0,};

	if(!priv->client)
	{
		printk("failure to get i2c_client.\n");
		return -EFAULT;
	}

	rv = read_temperature(priv->client, data);
	if(rv)
	{
		dev_err(priv->dev, "read_temperature failure.\n");
	}

	data[3] = data[1];
	data[2] = data[0];

	rv = read_humidity(priv->client, data);
	if(rv)
	{
		dev_err(priv->dev, "read_humidity failure.\n");
	}

	//printk("test %x %x %x %X \n", data[3], data[2], data[1], data[0]);

	return sprintf(buf, "temperature=%d humidity=%d\n", ((data[3] << 8) | data[2]), ((data[1] << 8) | data[0]) );//1000倍
}

/* @description: sysfs - echo写入属性函数
 *
 * @parm  : dev - 设备指针,创建file时候会指定dev
 * @parm  : attr - 设备属性,创建时候传入
 * @parm  : buf - 用户空间的buf
 * @parm  : count - 传入buf的size
 * @return: 写入的buf大小
 */
static ssize_t sht20_temp_humi_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char k_buf[10] = {0,};
	snprintf(k_buf, sizeof(k_buf), "%s", buf);

	dev_info(dev, "Don't echo to me  -> [%s] size [%d]\n", k_buf, count);

	return count;
}

//初始化属性值
static DEVICE_ATTR(sht20_temp_humi, 0644, sht20_temp_humi_show, sht20_temp_humi_store);

/* @description: i2c驱动probe函数，设备和驱动匹配后执行
 *
 * @parm  : client - i2c设备
 * @parm  : id - i2c设备ID
 * @return: 0 successfully , !0 failure
 */
static int sht20_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sht20_priv *priv = NULL;
	//struct device *dev;
	dev_t devno; //设备号
	int rv = 0;

	//0.给priv分配空间
	priv = devm_kzalloc(&client->dev, sizeof(struct sht20_priv), GFP_KERNEL);
	if(!priv)
	{
		return -ENOMEM;
	}

	//1.创建设备号
	if(0 != dev_major)
	{
		devno = MKDEV(dev_major, 0);
		rv = register_chrdev_region(devno, 1, DEV_NAME); //静态创建
	}
	else
	{
		rv = alloc_chrdev_region(&devno, 0, 1, DEV_NAME);//动态创建
		dev_major = MAJOR(devno);//获主设备号
	}

	if(rv < 0)
	{
		dev_err(&client->dev, "%s driver can't get major %d\n", DEV_NAME, dev_major);
		return rv;
	}

	//2.注册字符设备
	cdev_init(&priv->cdev, &sht20_fops); //初始化cdev
	priv->cdev.owner = THIS_MODULE;

	rv = cdev_add(&priv->cdev, devno, 1);
	if(0 != rv)
	{
		dev_err(&client->dev, "error %d add %s device failure.\n", rv, DEV_NAME);
		goto undo_major;
	}

	//3.创建类，驱动进行节点创建
	priv->dev_class = class_create(THIS_MODULE, DEV_NAME);
	if(IS_ERR(priv->dev_class))
	{
		dev_err(&client->dev, "%s driver create class failure.\n", DEV_NAME);
		rv = -ENOMEM;
		goto undo_cdev;
	}

	//4.创建设备 
	priv->dev = device_create(priv->dev_class, NULL, devno, NULL, DEV_NAME);
	if(IS_ERR(priv->dev))
	{
		rv = -ENOMEM;
		goto undo_class;
	}

	//5. 创建sys 属性 在platform下
	if(device_create_file(priv->dev, &dev_attr_sht20_temp_humi))
	{
		rv = -ENOMEM;
		goto undo_device;
	}

	//6. 保存私有数据
	priv->client = client;
	i2c_set_clientdata(client, priv);
	dev_set_drvdata(priv->dev, priv);

	dev_info(&client->dev, "sht20 i2c driver probe okay.\n");
	return 0;

undo_device:
	device_destroy(priv->dev_class, devno);

undo_class:
	class_destroy(priv->dev_class);

undo_cdev:
	cdev_del(&priv->cdev);

undo_major:
	unregister_chrdev_region(devno, 1);

	devm_kfree(&client->dev, priv);
	return rv;
}


/* @description: i2c驱动的remove函数，移除时候执行
 *
 * @parm  : client - i2c 
 * @parm  : 
 * @return: 0 successfully , !0 failure
 */
static int sht20_remove(struct i2c_client *client)
{
	struct sht20_priv *priv = i2c_get_clientdata(client);
	dev_t devno = MKDEV(dev_major, 0);

	//删除sys中的属性
	device_remove_file(priv->dev, &dev_attr_sht20_temp_humi);

	//设备销毁
	device_destroy(priv->dev_class, devno);

	//注销类
	class_destroy(priv->dev_class);

	//删除字符设备
	cdev_del(&priv->cdev);
	unregister_chrdev_region(devno, 1);

	//释放堆
	devm_kfree(&client->dev, priv);
	dev_info(&client->dev, "sht20 driver remove.\n");

	return 0;
}

//传统方式ID列表
static const struct i2c_device_id sht20_id[] = {
	{"sht20", 0},
	{}
};

//设备树匹配列表
static const struct of_device_id of_sht20_match [] = {
	{.compatible="sht20"},
	{}
};
MODULE_DEVICE_TABLE(of, of_sht20_match);

static struct i2c_driver sht20_drv = {
	.probe = sht20_probe,
	.remove = sht20_remove,
	.driver = {
		.name = "sht20 driver 0.1", //name 不重要
		.owner = THIS_MODULE,
		.of_match_table = of_sht20_match,
	},
	.id_table = sht20_id,
};

//注册sht20驱动
module_i2c_driver(sht20_drv);

MODULE_AUTHOR("rsh");
MODULE_DESCRIPTION("i.MX6ULL sht20 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:imx_sht20_i2c_driver");
