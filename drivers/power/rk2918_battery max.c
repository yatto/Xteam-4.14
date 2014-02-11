/* drivers/power/rk2918_battery.c
 *
 * battery detect driver for the rk2918 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>

//#define BATTERY_DEBUG

#ifdef BATTERY_DEBUG
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif

int rk29_battery_dbg_level = 0;

#define SPENDCNT_IN_OFF_STATE	(3600)

#define BAT_ADC_TABLE_LEN       11
static int adc_raw_table_bat[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_RK29_ACH7)
	3500, 3579, 3649, 3676, 3694, 3731, 3789, 3856, 3927, 4007, 4150
#elif defined(CONFIG_MACH_A8)
    3530, 3597, 3628, 3641, 3660, 3697, 3747, 3809, 3879, 3945, 4050
#elif defined(CONFIG_MACH_M908)
    3490, 3585, 3662, 3685, 3700, 3736, 3792, 3860, 3925, 4010, 4070
#elif defined(CONFIG_MACH_A7HTC) || defined(CONFIG_MACH_A70HT3R) || defined(CONFIG_MACH_A7HC) || defined(CONFIG_MACH_A70HT3N)
	3490, 3648, 3693, 3721, 3740, 3762, 3800, 3860, 3920, 3975, 4050
#elif defined (CONFIG_MACH_M726HN)
	3500, 3585, 3642, 3662, 3682, 3712, 3761, 3820, 3885, 3962, 4050
#elif defined (CONFIG_MACH_M726)
	//3500, 3535, 3585, 3635, 3685, 3735, 3785, 3828, 3870, 3925, 4070
	3500, 3596, 3647, 3676, 3696, 3730, 3780, 3846, 3916, 3990, 4065
#elif defined (CONFIG_MACH_M803) || defined (CONFIG_MACH_M803HD)
	3500, 3650, 3715, 3738, 3765, 3805, 3857, 3923, 3977, 4030, 4120
#elif defined (CONFIG_MACH_M900) || defined (CONFIG_MACH_M900HD) || defined (CONFIG_MACH_M900HDW)
	3500, 3597, 3650, 3670, 3691, 3723, 3764, 3822, 3886, 3967, 4050
#elif defined (CONFIG_MACH_M911)
	3500, 3581, 3636, 3656, 3681, 3716, 3763, 3832, 3905, 3990, 4080
#elif defined(CONFIG_MACH_A7ECHR)
	3500, 3539, 3560, 3588, 3638, 3692, 3760, 3835, 3910, 3990, 4080
#elif defined (CONFIG_MACH_M732)
	3500, 3578, 3635, 3653, 3673, 3709, 3765, 3829, 3898, 3980, 4065
#elif defined (CONFIG_MACH_M722)
	3500, 3625, 3675, 3708, 3728, 3748, 3795, 3855, 3915, 3990, 4080
#elif defined (CONFIG_MACH_M1005)
	6800, 6960, 7090, 7220, 7300, 7420,	7530, 7770, 7820, 8000, 8200
#else
    3500, 3579, 3649, 3676, 3694, 3731, 3789, 3856, 3927, 4007, 4150
#endif
};

static int adc_raw_table_ac[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_RK29_ACH7)
	3691, 3760, 3800, 3827, 3845, 3885, 3950, 4007, 4078, 4158, 4300//4185//4301
#elif defined(CONFIG_MACH_A8)
    3680, 3747, 3778, 3791, 3810, 3847, 3897, 3959, 4029, 4095, 4210//4185//4301
#elif defined(CONFIG_MACH_M908)
	3680, 3795, 3875, 3895, 3910, 3946, 3996, 4065, 4115, 4190, 4240
#elif defined(CONFIG_MACH_A7HTC) || defined(CONFIG_MACH_A70HT3R) || defined(CONFIG_MACH_A7HC) || defined(CONFIG_MACH_A70HT3N)
	3640, 3760, 3793, 3823, 3845, 3865, 3900, 3960, 4020, 4075, 4110
#elif defined (CONFIG_MACH_M726HN)
	3690, 3815, 3860, 3875, 3895, 3920, 3965, 4020, 4078, 4158, 4300
#elif defined (CONFIG_MACH_M726)
	3710, 3795, 3840, 3868, 3887, 3925, 3970, 4030, 4100, 4160, 4220
#elif defined (CONFIG_MACH_M803) || defined (CONFIG_MACH_M803HD)
	3650, 3795, 3857, 3880, 3910, 3950, 3998, 4060, 4110, 4160, 4240
#elif defined (CONFIG_MACH_M900) || defined (CONFIG_MACH_M900HD) || defined (CONFIG_MACH_M900HDW)
	3700, 3795, 3835, 3855, 3875, 3905, 3950, 4002, 4060, 4135, 4220
#elif defined (CONFIG_MACH_M911)
	3710, 3790, 3840, 3860, 3885, 3918, 3965, 4035, 4100, 4175, 4250
#elif defined(CONFIG_MACH_A7ECHR)
	3715, 3750, 3775, 3800, 3848, 3895, 3965, 4040, 4105, 4180, 4250
#elif defined (CONFIG_MACH_M732)
	3710, 3780, 3845, 3885, 3900, 3925, 3975, 4050, 4050, 4170, 4240
#elif defined (CONFIG_MACH_M722)
	3710, 3825, 3876, 3910, 3930, 3950, 3995, 4050, 4110, 4180, 4250
#elif defined (CONFIG_MACH_M1005)
	7000, 7160, 7290, 7420, 7500, 7620, 7730, 7970, 8020, 8200, 8400
#else
    3691, 3780, 3845, 3885, 3900, 3925, 3975, 4050, 4110, 4170, 4300//4185//4301
#endif
};


#ifdef RK29_USB_CHARGE_SUPPORT
static int adc_raw_table_usb[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_A7HTC) || defined(CONFIG_MACH_A70HT3R)
    //3650, 3690, 3720, 3635, 3750, 3787, 3800,3850, 3899, 3970, 4030//4185//4301
    3560, 3700, 3740, 3770, 3790, 3810, 3850, 3908, 3970, 4025, 4090
#elif defined(CONFIG_MACH_A8)
	3580, 3645, 3675, 3691, 3710, 3750, 3800, 3860, 3930, 3995, 4010
#elif defined (CONFIG_MACH_M900) || defined (CONFIG_MACH_M900HD) || defined (CONFIG_MACH_M900HDW)
	3580, 3665, 3718, 3740, 3762, 3795, 3830, 3895, 3955, 4035, 4120
#endif
};
#endif

#ifndef	RK29_ADC_CLI_VALUE
#define RK29_ADC_CLI_VALUE		0
#endif

#ifndef RK29_ADC_CLI_ADD
#define RK29_ADC_CLI_ADD		0
#endif

static int gBatFullFlag =  0;
static int shutdownvoltage = 3400;

static int gBatLastStatus = 0;
static int gBatStatus =  POWER_SUPPLY_STATUS_UNKNOWN;
static int gBatHealth = POWER_SUPPLY_HEALTH_GOOD;
static int gBatLastPresent = 0;
static int gBatPresent = 1;
static int gBatLastVoltage =  0;
static int gBatVoltage =  BATT_NOMAL_VOL_VALUE;
static int gBatLastCapacity = 0;
static int gBatLastChargeCapacity = 100;          //记录充电时的电池容量
static int gBatCapacity = ((BATT_NOMAL_VOL_VALUE-BATT_ZERO_VOL_VALUE)*100/(BATT_MAX_VOL_VALUE-BATT_ZERO_VOL_VALUE));
static int gLastBatCapacity = ((BATT_NOMAL_VOL_VALUE-BATT_ZERO_VOL_VALUE)*100/(BATT_MAX_VOL_VALUE-BATT_ZERO_VOL_VALUE));
static int gBatStatusChangeCnt = 0;
static int gBatCapacityUpCnt = 0;
static int gBatCapacityDownCnt = 0;
static int gBatHighCapacityChargeCnt = 0;
static int gBatUsbChargeFlag = 0;
static int gBatUsbChargeCnt = 0;
static int shutdownflag = 0;

static int gBatVoltageSamples[NUM_VOLTAGE_SAMPLE+2]; //add 2 to handle one bug
static int gBatSlopeValue = 0;
static int gBatVoltageValue[2]={0,0};
static int *pSamples = &gBatVoltageSamples[0];		//采样点指针
static int gFlagLoop = 0;		//采样足够标志
//static int gNumSamples = 0;
static int gNumCharge = 0;
static int gMaxCharge = 0;
static int gNumLoader = 0;
static int gMaxLoader = 0;

static struct regulator *pChargeregulator;
static int gBatChargeStatus =0;
static int gUsbCharge = 0;
static int gBatStatusBack =  POWER_SUPPLY_STATUS_UNKNOWN;

static int openfailflag = 0;
static uint8_t openfailcount = 20;  //  文件打开失败的话，重复次数

unsigned long  batteryspendcnt = 0;
unsigned long  last_batteryspendcnt = 0;
static int usb_charging_on = 0;

static int last_BatChargeStatus = 0;

extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);

struct rk2918_battery_data {
	int irq;
	spinlock_t lock;
	
	struct delayed_work	work;
	struct workqueue_struct *wq;
	
	struct work_struct 	timer_work;
	struct timer_list timer;
	struct power_supply battery;
	
#ifdef RK29_USB_CHARGE_SUPPORT
	struct power_supply usb;
#endif
	struct power_supply ac;
    
    int dc_det_pin;
    int batt_low_pin;
    int charge_set_pin;
	int charge_ok_pin;

    int dc_det_level;
    int batt_low_level;
    int charge_set_level;
	int charge_ok_level;
	
    int dc_det_irq;

	int adc_bat_divider;
	int bat_max;
	int bat_min;
	int adc_val;
	
	int full_times;
	
	struct adc_client *client; 
};


/* temporary variable used between rk2918_battery_probe() and rk2918_battery_open() */
static struct rk2918_battery_data *gBatteryData;

enum {
	BATTERY_STATUS          = 0,
	BATTERY_HEALTH          = 1,
	BATTERY_PRESENT         = 2,
	BATTERY_CAPACITY        = 3,
	BATTERY_AC_ONLINE       = 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED   	= 6,
	BATTERY_INT_STATUS	    = 7,
	BATTERY_INT_ENABLE	    = 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;


#define BATT_FILENAME "/data/bat_last_capacity.dat"
#include <linux/fs.h>

#define DATE_FILENAME "/data/time_record.dat"
static void save_spendcnt_to_datefile()
{
	struct file* fp;
	int buf_len;
	char _tmp_buf[20];
	char value[20];
	char *p = value;
	unsigned long spendcnt_poweroff = 0;

	fp = filp_open(DATE_FILENAME,O_WRONLY | O_CREAT,0);
	if(IS_ERR(fp))
    {
		printk("bryan---->open file /data/time_record.dat failed\n");
	}
	else
	{
		sprintf(_tmp_buf, "%lu", batteryspendcnt);
		buf_len = strlen(_tmp_buf);
		kernel_write(fp, _tmp_buf, buf_len, 0);		
	    filp_close(fp,NULL);
	}
}

static unsigned long get_spendcnt_from_datefile()
{
	struct file* fp;
	int buf_len;
	char _tmp_buf[20];
	char value[20];
	char *p = value;
	unsigned long spendcnt_poweroff = 0;

	//buf_len = strlen(buf);
	fp = filp_open(DATE_FILENAME,O_RDONLY,0);
	if(IS_ERR(fp))
    {
		printk("bryan---->open file /data/time_record.dat failed\n");
		return spendcnt_poweroff;
	}
	else
	{
		sprintf(_tmp_buf, "%lu", batteryspendcnt);
		buf_len = strlen(_tmp_buf);
		kernel_write(fp, _tmp_buf, buf_len, 0);

		kernel_read(fp,0,value,10);
	    filp_close(fp,NULL);

		value[20]=0;
		while(*p)
		{
		    if(*p==0x0d)
		    {
				*p=0;
				break;
			}
			p++;
		}	
		sscanf(value,"%d",&spendcnt_poweroff);
		printk("bryan---->last poweroff_spendcnt = %d\n",spendcnt_poweroff);
	}
	return spendcnt_poweroff;
}

static void rk2918_batscan_timer(unsigned long data);

static ssize_t rk2918_battery_startget_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	static int only_one = 0;

	if (only_one != 0)
	{
		return len;
	}

	only_one = 1;
	

	if ((*buf=='A'))
	{
		rk2918_batscan_timer(NULL);

	}
	else
	{

		printk("%s error %s %d",__func__,buf,len);
		//rk2918_batscan_timer(NULL);
	}

	return len;

}

static DEVICE_ATTR(startget,0666,NULL,rk2918_battery_startget_store);
static int rk2918_get_bat_capacity_raw(int BatVoltage);
int lastlost = 0;
static int rk2918_battery_load_capacity(void)
{
    int i;
    int tmp = 0;
	int  loadcapacity = 0;
	int  truecapacity = 0;
    char value[11];
    static char lastthree[6]= {0};
	char* p = value;
	unsigned long spendcnt_intervals = 0;
	ktime_t ktimetmp; 
	
	ktimetmp = ktime_get_real();
    struct file* fp = filp_open(BATT_FILENAME,O_RDONLY,0);
    
    //get true capacity
    for (i = 0; i < 50; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 50;
    tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
	
	if(RK29_ADC_CLI_ADD)
		tmp += RK29_ADC_CLI_VALUE;
	else
		tmp -= RK29_ADC_CLI_VALUE;
	
    truecapacity = rk2918_get_bat_capacity_raw(tmp);
    
	if(IS_ERR(fp))
    {
		printk("hj---->open file /data/bat_last_capacity.dat failed\n");
		printk("truecapacity = %d\n", truecapacity);
		
		if(truecapacity>=100)
			truecapacity = 100;
	 	if(truecapacity==0)
			truecapacity=1;
			
		openfailflag = 1;

		if (openfailcount <= 5)
		{
			lastthree[openfailcount-1] = truecapacity;

			if (openfailcount == 1)
			{
				tmp = 0;
				for (i=0;i<5;i++)
				{
					tmp += lastthree[4-i];	
					printk("%s...............%d\n",__func__,tmp);
				}

				truecapacity = tmp/5;
				printk("%s...............%d\n",__func__,tmp);

			}
		}
		return truecapacity;
	}
	else
	{
		openfailflag = 0;
		openfailcount = 0;
	}
	kernel_read(fp,0,value,10);
    filp_close(fp,NULL);

	value[10]=0;
	while(*p)
	{
	    if(*p==0x0d)
	    {
			*p=0;
			break;
		}
		p++;
	}	

	//spendcnt_intervals = ktimetmp.tv.sec - get_spendcnt_from_datefile();
	sscanf(value,"%d",&loadcapacity);
	printk("hj---->loadcapacity = %d, truecapacity = %d\n",loadcapacity, truecapacity);
	if((loadcapacity < 0) || (loadcapacity > 100))
		loadcapacity = truecapacity;
	// will show capacity in last poweroff if not more than 6 houre in off state
#if !(defined (CONFIG_MACH_M900) || defined (CONFIG_MACH_M900HD) || defined (CONFIG_MACH_M900HDW))
	if(( spendcnt_intervals < SPENDCNT_IN_OFF_STATE) && ((truecapacity-loadcapacity) < 25))
	{			
		if (loadcapacity == 0)
		{
			loadcapacity = 1;
		}
		else
		{
			if (loadcapacity >= 100)
			{
				loadcapacity = 100;
			}
		}	
		return loadcapacity;
	}
	printk("off state more than 1 housr\n");
#endif
	//如果从文件中读取的电压比实际的高很多的话，说明是长时间放置导致放电
	if (loadcapacity > truecapacity)
	{
	    if (loadcapacity - truecapacity > 20)
	    {
	        loadcapacity = truecapacity;
	    }
	}
	else
	{
		if ( ((truecapacity-loadcapacity) >= 20))
		{

			if (truecapacity < 30)
			{
				if (loadcapacity < 10)
				{
					loadcapacity = truecapacity/2;
				}
			}
			else
			{

				loadcapacity = truecapacity;	
				
			}

		}


	}
	    
	if (loadcapacity == 0)
	{
		loadcapacity = 1;
	}
	else
	{
		if (loadcapacity >= 100)
		{
			loadcapacity = 100;
		}
	}
	
	lastlost = loadcapacity;
	return loadcapacity;
}

static void rk2918_charge_enable(void)
{
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(gBatteryData->charge_set_pin, gBatteryData->charge_set_level);
    }
}

static void rk2918_charge_disable(void)
{
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(gBatteryData->charge_set_pin, 1 - gBatteryData->charge_set_level);
    }
}

extern int suspend_flag;
static void rk2918_get_charge_status(void)
{
    int charge_on = 0;
    
    if (gBatteryData->dc_det_pin != INVALID_GPIO)
    {
        if (gpio_get_value (gBatteryData->dc_det_pin) == gBatteryData->dc_det_level)
        {
            charge_on = 1;
        }
    }
    DBG("charge_on status: %d \n", charge_on);
#ifdef RK29_USB_CHARGE_SUPPORT
    if (charge_on == 0)
    {
        if (suspend_flag) return;
            
        if (1 == dwc_vbus_status())         //检测到USB插入，但是无法识别是否是充电器
        {                                   //通过延时检测PC识别标志，如果超时检测不到，说明是充电
            if (0 == get_msc_connect_flag())
            {                               //插入充电器时间大于一定时间之后，开始进入充电状态
				if (++gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES)
                {
                    gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
                    charge_on = 1;
                }
            }                               //否则，不进入充电模式
            #if defined(CONFIG_MACH_RK29_ACH8)
            	charge_on = 1;
            #endif
        }                   
        else
        {
            gBatUsbChargeCnt = 0;
            if (2 == dwc_vbus_status()) 
            {
                charge_on = 1;
            }
        }
    }
#endif
        
    if (charge_on)
    {
        if(gBatChargeStatus !=1) 
        {            
            gBatChargeStatus = 1;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            rk2918_charge_enable();
        }
    } 
    else 
    {
        if(gBatChargeStatus != 0) 
        {
            gBatChargeStatus = 0;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            rk2918_charge_disable();
        }
    }
}

static void rk2918_get_bat_status(struct rk2918_battery_data *bat)
{
	static int stable = 0;
	
  	rk2918_get_charge_status(); 
	if((gBatChargeStatus == 1))
	{
        if (gBatteryData->charge_ok_pin == INVALID_GPIO)
        {
            printk("dc_det_pin invalid!\n");
            return;
        }
    
			if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
			{
				return;
			}
	    if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity >= 80))
        {
			gBatteryData->full_times++;
			stable = 0;
            if (gBatteryData->full_times >= NUM_CHARGE_FULL_DELAY_TIMES)
		    {
				gBatStatus = POWER_SUPPLY_STATUS_FULL;
				gBatStatusBack=POWER_SUPPLY_STATUS_FULL;
		        gBatteryData->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
		    }
		    else
		    {
		        gBatStatus = POWER_SUPPLY_STATUS_CHARGING;
		    }
	    }
	    else
	    {
				if (gBatteryData->full_times>0)
				{
	
					stable++;
					if(stable >= NUM_STABLE_SAMPLE)
					{
						stable = 0;
						gBatteryData->full_times = 0;
					
					}
	
				}
	    	    gBatStatus = POWER_SUPPLY_STATUS_CHARGING;			
	        
		}
	}
	else 
    	{
    	stable = 0;	
	    gBatteryData->full_times = 0;
        gBatStatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static void rk2918_get_bat_health(struct rk2918_battery_data *bat)
{
	gBatHealth = POWER_SUPPLY_HEALTH_GOOD;
}

static void rk2918_get_bat_present(struct rk2918_battery_data *bat)
{
	if(gBatVoltage < bat->bat_min)
		gBatPresent = 0;
	else
		gBatPresent = 1;
}

int capacitytmp = 0;
static int rk2918_get_bat_capacity_raw(int BatVoltage)
{
    int i = 0;
	int capacity = 0;
	int *p = adc_raw_table_bat;
#ifdef RK29_USB_CHARGE_SUPPORT
	if(gpio_get_value(RK29_PIN0_PA0)==0)//有USB插入。。
	{
		//printk("====>USB inster, gBatChargeStatus = %d  \n", gBatChargeStatus);
		if(0 == get_msc_connect_flag()) //(gBatChargeStatus == 1)
		{
			p = adc_raw_table_ac;
			usb_charging_on = 0;
		}
		else
		{
			p = adc_raw_table_usb;
			usb_charging_on = 1;
		}
	}
#else 
    if (gBatChargeStatus == 1)
    {
        p = adc_raw_table_ac;
    }
#endif
	
	if(BatVoltage >= p[BAT_ADC_TABLE_LEN - 1])
	{
	    //当电压超过最大值
	    capacity = 100;
	}	
	else if(BatVoltage <= p[0])
	{
	    //当电压低于最小值
	    capacity = 0;
	}
	else
	{
    	//计算容量
    	for(i = 0; i < BAT_ADC_TABLE_LEN - 1; i++)
        {
    		
    		if((p[i] <= BatVoltage) && (BatVoltage < p[i+1]))
    		{
    			capacity = i * 10 + ((BatVoltage - p[i]) * 10) / (p[i+1] - p[i]);
    			break;
    		}
    	}
    }	
	//DBG("bryan-->BatVoltage = %d, capacity = %d\n", BatVoltage, capacity);
    return capacity;
}

static int rk2918_battery_resume_get_Capacity(int deltatime)
{
	int i;
    int tmp = 0;
    int capacity = 0;
	gFlagLoop = 0;
	pSamples = &gBatVoltageSamples[0];
    for (i = 0; i < 80; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 80;
    tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
	if(RK29_ADC_CLI_ADD)
		tmp += RK29_ADC_CLI_VALUE;
	else
		tmp -= RK29_ADC_CLI_VALUE;
    capacity = rk2918_get_bat_capacity_raw(tmp);
	
    if (gBatChargeStatus == 1)
    {
    	/*
        if (deltatime > (100 - gBatCapacity) * CHARGE_MIN_SECOND)
            deltatime = (100 - gBatCapacity) * CHARGE_MIN_SECOND;
        if (capacity > gBatCapacity + (deltatime / CHARGE_MIN_SECOND))       //采样电池容量偏差较大，将容量拉回
        {
            capacity = gBatCapacity + (deltatime / CHARGE_MIN_SECOND);
        }
		*/
		//
		if(gBatCapacity > 80)
		{
			if(gBatChargeStatus == last_BatChargeStatus)
				capacity = gBatCapacity + deltatime/CHARGE_MID_SECOND;
		}
		else
		{
			/*some battery cannot arrive to 4.1V when charging full*/
			if((deltatime > (100 - gBatCapacity) * CHARGE_MID_SECOND) && (gBatChargeStatus == last_BatChargeStatus))
			{
				capacity = 99;
				printk("0000rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, capacity = %d\n", 
           			last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, capacity);
				if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
				{
					capacity = 100;
				}
   				return capacity;
			}
			/*if battery is not full after long charging*/
			if((capacity > 80)&& (gBatChargeStatus == last_BatChargeStatus))
			{
				if(capacity < gBatCapacity + deltatime/CHARGE_MID_SECOND)
					capacity = gBatCapacity + deltatime/CHARGE_MID_SECOND;
				if(capacity >= 100)
					capacity = 99;
				if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
				{
					capacity = 100;
				}
				printk("11111rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, capacity = %d\n", 
           			last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, capacity);
				return capacity;
			}
			
			if(capacity > gBatCapacity)
	        {
	        	if((capacity-gBatCapacity)>15)
					gBatCapacity = capacity;
				else
					 capacity=gBatCapacity ;
	        }
	        else if (capacity < gBatCapacity)
	        {
	        	if((gBatCapacity - capacity)>10)
					gBatCapacity = capacity;
				else
					 capacity=gBatCapacity ;
	        }
		}
		if (capacity >= 100) 
			capacity = 99;
    }
    else
    {
    	/*
        if (deltatime > gBatCapacity * DISCHARGE_MIN_SECOND)
            deltatime = gBatCapacity * DISCHARGE_MIN_SECOND;            
        if (capacity < gBatCapacity - (deltatime / DISCHARGE_MIN_SECOND))    //采样电池容量偏差较大，将容量拉回
        {
            capacity = gBatCapacity - (deltatime / DISCHARGE_MIN_SECOND);
        }
		*/
		
        if (capacity < gBatCapacity)
        {
			if((gBatCapacity - capacity)>10)
				gBatCapacity = capacity;
			else
				 capacity=gBatCapacity ;
        }
        else if (capacity > gBatCapacity)
        {
        	if((capacity-gBatCapacity)>15)
				gBatCapacity = capacity;
			else
				 capacity=gBatCapacity ;
        }
    }
	
    if (capacity == 0) 
		capacity = 1;
    if (capacity >= 100) 
		capacity = 100;
    
    if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
	{
	    capacity = 100;
	}
		
    printk("rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, ktmietmp.tv.sec = %lu, capacity = %d\n", 
           last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, batteryspendcnt, capacity);
    
    return capacity;
}

static int rk2918_get_bat_capacity_ext(int BatVoltage)
{
    int i = 0;
	int capacity = 0;
	static int lostcount = 0;
	uint8_t speedlose = 1;
	
    capacity = rk2918_get_bat_capacity_raw(BatVoltage);
	capacitytmp = capacity;
	
	//充放电状态变化后，Buffer填满之前，不更新
    if (gBatStatusChangeCnt < NUM_VOLTAGE_SAMPLE)
    {
        capacity = gBatCapacity;
    }
        
    if (gBatChargeStatus == 1)
    {
		if ((capacity > gBatCapacity) && (gBatCapacity < 100) && (gBatCapacityDownCnt == 0))
        {
            capacity = gBatCapacity + 1;
			if(usb_charging_on)
            	gBatCapacityDownCnt = NUM_CHARGE_MIN_SAMPLE + NUM_CHARGE_MIN_SAMPLE;
			else
				gBatCapacityDownCnt = NUM_CHARGE_MIN_SAMPLE;
            gBatHighCapacityChargeCnt = 0;
        }
        else if (((capacity <= gBatCapacity) && (gBatCapacity < 100)) || (gBatCapacity >= 90))
        {
        	if(usb_charging_on)
        	{
	            if (++gBatHighCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE + NUM_CHARGE_MID_SAMPLE)
	            {
	                //长时间内充电电压无变化，开始计时充电
	                capacity = gBatCapacity + 1;
	                gBatHighCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
	            }
	            else
	            {
	                capacity = gBatCapacity;
	            }
        	}
			else
			{
	            if (++gBatHighCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE)
	            {
	                //长时间内充电电压无变化，开始计时充电
	                capacity = gBatCapacity + 1;
	                gBatHighCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
	            }
	            else
	            {
	                capacity = gBatCapacity;
	            }
        	}
				
        }
        else
        {
            capacity = gBatCapacity;
        }
        
		if (capacity == 0)
		{
		    capacity = 1;
		}
		
		if (capacity >= 100)
		{
		    capacity = 99;
		}
		
		if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
		{
		    capacity = 100;
		}
    }    
    else
    {   
        //放电时,只允许电压下降
        if (capacity > gBatCapacity)
        {
            capacity = gBatCapacity;
        }
        
        if ((capacity < gBatCapacity) && (gBatCapacityDownCnt == 0))
        {
            capacity = gBatCapacity - 1;
            gBatCapacityDownCnt = NUM_DISCHARGE_MIN_SAMPLE;
        }
        else
        {
            capacity = gBatCapacity;
        }
		
		if (lostcount++ > NUM_SPEEDLOSE_SAMPLE)
 		{
			lostcount = 0;
			if (((lastlost-gBatVoltage)>8)&&(gBatVoltage >= 3400))// start play game
			{
				shutdownvoltage = 3300;	
				printk("%s...lastlost=%d.enter game\n",__func__,gBatVoltage);
			}
			else
			{
				if ((gBatVoltage-lastlost)>8) //exit game
				{
					shutdownvoltage = 3400;
					printk("%s.. lastlost=%d..exit game\n",__func__,gBatVoltage);
				}
			}
			lastlost = gBatVoltage;
			

 		}

// <8%capacity enter game will make mistake
 		if (((gBatVoltage<=shutdownvoltage) && (lastlost-gBatVoltage)<=3) || (gBatVoltage <= 3300))
 		{	
 			shutdownflag++;
 			if(shutdownflag >= NUM_SHUTD0WN_SAMPLE)
			{
				capacity = 0;
				printk("%s.........%d.........%d\n",__func__,__LINE__,shutdownflag);
				shutdownflag = NUM_SHUTD0WN_SAMPLE+1;
			}
 		}	
    }
    
    if (gBatCapacityDownCnt > 0)
    {
        gBatCapacityDownCnt --;
    }
    
	return capacity;
}

unsigned long AdcTestvalue = 0;
unsigned long AdcTestCnt = 0;
static void rk2918_get_bat_voltage(struct rk2918_battery_data *bat)
{
	int value;
	int i,*pStart = &gBatVoltageSamples[0],num = 0;
	int temp[2] = {0,0};
	
	value = gBatteryData->adc_val;
	AdcTestvalue = value;
    adc_async_read(gBatteryData->client);
	if(RK29_ADC_CLI_ADD)
		*pSamples++ = (value * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R) + RK29_ADC_CLI_VALUE;
	else
		*pSamples++ = (value * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R) - RK29_ADC_CLI_VALUE;
	//*pSamples++ = (value * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
	num = pSamples - pStart;
	if (num >= NUM_VOLTAGE_SAMPLE)
	{
	    pSamples = pStart;
	    gFlagLoop = 1;
	}
	if (gFlagLoop == 1)
	{
	    num = NUM_VOLTAGE_SAMPLE;
	}
	value = 0;
	for (i = 0; i < num; i++)
	{
	    value += gBatVoltageSamples[i];
	}
	gBatVoltage = value / num;
	//gBatVoltage = (value * BAT_2V5_VALUE * 2) / 1024;
	
	/*消除毛刺电压*/
	if(gBatVoltage >= BATT_MAX_VOL_VALUE + 10)
		gBatVoltage = BATT_MAX_VOL_VALUE + 10;
	else if(gBatVoltage <= BATT_ZERO_VOL_VALUE - 10)
		gBatVoltage = BATT_ZERO_VOL_VALUE - 10;

    //充放电状态变化时,开始计数	
	if (++gBatStatusChangeCnt > NUM_VOLTAGE_SAMPLE)  
	    gBatStatusChangeCnt = NUM_VOLTAGE_SAMPLE + 1;
}

int first_flag = 1;
static void rk2918_get_bat_capacity(struct rk2918_battery_data *bat)
{
    int deltatime = 0;
    ktime_t ktmietmp;
    struct timespec ts;
    
	ktmietmp = ktime_get_real();
  	ts = ktime_to_timespec(ktmietmp);
	deltatime = ts.tv_sec - batteryspendcnt;
	//deltatime = ktmietmp.tv.sec - batteryspendcnt;

	if (first_flag || (openfailflag && (openfailcount > 1))) 
	//多次打开，刚开始采样时不准，多次采样。。
	{
	     if(first_flag == 1)
	     		first_flag--;
	     openfailcount--;
	  printk("%s,first_flag=%d,openfailflag=%d,openfailcount=%d\n",__func__,first_flag,openfailflag,openfailcount);   		
	    gBatCapacity = rk2918_battery_load_capacity();
	    if (gBatCapacity == 0) 
			gBatCapacity = 1;
	}

	else if ((deltatime > 600) && (first_flag == 0))//处理休眠之后的电量回复,如果超过十分钟
	{		
        	gBatCapacity = rk2918_battery_resume_get_Capacity(deltatime);
	}
	else
	{
        	gBatCapacity = rk2918_get_bat_capacity_ext(gBatVoltage);
        /*	
        	if(deltatime>1)
        	{
			if(gBatCapacity--<2)
				gBatCapacity  = 50;

			if(gBatCapacity==15)
				gBatCapacity = 4;
			batteryspendcnt = ktmietmp.tv.sec;
        	}
        */
        
    }
    //batteryspendcnt = ktmietmp.tv.sec;
    batteryspendcnt=ts.tv_sec;
	
}

#define BATT_DBG_FILE "/flash/bat_dbg_record.dat"
static int buf_offset = 0;
static void rk2918_battery_timer_work(struct work_struct *work)
{		
	rk2918_get_bat_status(gBatteryData);
	rk2918_get_bat_health(gBatteryData);
	rk2918_get_bat_present(gBatteryData);
	rk2918_get_bat_voltage(gBatteryData);
	//to prevent gBatCapacity be changed sharply
	if (gBatCapacity < 0)
	{
		gBatCapacity = 0;
	}
	else
	{
		if (gBatCapacity > 100)
		{
			gBatCapacity = 100;
		}
	}
	rk2918_get_bat_capacity(gBatteryData);
	
	if (rk29_battery_dbg_level)
	{
    	if (++AdcTestCnt >= 20)
    	{
    	    AdcTestCnt = 0;
	   		printk("\ncharge_ok_level =%d, chg_ok= %d, gBatStatus = %d, adc_val = %d, TrueBatVol = %d,gBatVol = %d, gBatCap = %d, captmp = %d, sec = %lu\n", 
	            gBatteryData->charge_ok_level, gpio_get_value(gBatteryData->charge_ok_pin), gBatStatus, AdcTestvalue, 
	            ((AdcTestvalue * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R)), 
	            gBatVoltage, gBatCapacity, capacitytmp, batteryspendcnt);
    	}
    }

	/*update battery parameter after adc and capacity has been changed*/
	if((gBatStatus != gBatLastStatus) || (gBatPresent != gBatLastPresent) || (gBatCapacity != gBatLastCapacity))
	{
		if (rk29_battery_dbg_level)
		{
			char _tmp_buf[250];
			int buf_len = 0;
			struct file* fp;
			sprintf(_tmp_buf, "gBatStatus = %d, adc_val = %d, TrueBatVol = %d,gBatVol = %d, gBatCap = %d, captmp = %d, sec = %lu, inter_sec = %lu\n", 
		    	            gBatStatus, AdcTestvalue, ((AdcTestvalue * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R)), 
		    	            gBatVoltage, gBatCapacity, capacitytmp, batteryspendcnt, batteryspendcnt - last_batteryspendcnt);
			buf_len = strlen(_tmp_buf);
			fp = filp_open(BATT_DBG_FILE,O_RDWR | O_APPEND | O_CREAT, 0);
			if(IS_ERR(fp))
		    {
				printk("bryan---->open file /data/bat_dbg_record.dat failed\n");
			}
			else
			{
				kernel_write(fp, _tmp_buf, buf_len ,buf_offset);
			    filp_close(fp,NULL);
				buf_offset += buf_len; 
			}
			last_batteryspendcnt = batteryspendcnt;
		}
		//gNumSamples = 0;
		gBatLastStatus = gBatStatus;
		gBatLastPresent = gBatPresent;
		gBatLastCapacity = gBatCapacity;
		power_supply_changed(&gBatteryData->battery);
		save_spendcnt_to_datefile();
	}
	
}


static void rk2918_batscan_timer(unsigned long data)
{
    gBatteryData->timer.expires  = jiffies + msecs_to_jiffies(TIMER_MS_COUNTS);
	add_timer(&gBatteryData->timer);
	schedule_work(&gBatteryData->timer_work);	
}

#ifdef RK29_USB_CHARGE_SUPPORT
static int rk2918_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		printk("%s:%d\n",__FUNCTION__,val->intval);
		break;

	default:
		return -EINVAL;
	}
	
	return 0;

}
#endif

static int rk2918_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			if (gBatChargeStatus == 1)
				val->intval = 1;
			else
				val->intval = 0;	
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rk2918_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct rk2918_battery_data *data = container_of(psy,
		struct rk2918_battery_data, battery);
	int ret = 0;
	//DBG("\n");
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = gBatStatus;
		DBG("gBatStatus=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = gBatHealth;
		DBG("gBatHealth=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = gBatPresent;
		DBG("gBatPresent=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if(gBatVoltageValue[1] == 0)
		val ->intval = gBatVoltage;
		else
		val ->intval = gBatVoltageValue[1];
		DBG("gBatVoltage=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = gBatCapacity;
		DBG("gBatCapacity=%d%%\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = data->bat_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = data->bat_min;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	//DBG("\n");
	return ret;
}

static enum power_supply_property rk2918_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

#ifdef RK29_USB_CHARGE_SUPPORT
static enum power_supply_property rk2918_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

static enum power_supply_property rk2918_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


#ifdef CONFIG_PM
static int rk2918_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
#if defined(DC_CURRENT_IN_TWO_MODE)
	gpio_set_value(GPIO_CURRENT_CONTROL, 0);
#endif
	flush_scheduled_work();
	last_BatChargeStatus = gBatChargeStatus;
	return 0;
}

static int rk2918_battery_resume(struct platform_device *dev)
{
	/* things may have changed while we were away */
#if defined(DC_CURRENT_IN_TWO_MODE)
	gpio_set_value(GPIO_CURRENT_CONTROL, 1);
#endif
	schedule_work(&gBatteryData->timer_work);
	return 0;
}
#else
#define rk2918_battery_suspend NULL
#define rk2918_battery_resume NULL
#endif

static irqreturn_t rk2918_battery_interrupt(int irq, void *dev_id)
{
//    if ((rk2918_get_charge_status()) && (gBatFullFlag != 1))
//    {
//        gBatFullFlag = 1;
//    }

    return 0;
}

static irqreturn_t rk2918_dc_wakeup(int irq, void *dev_id)
{   
    gBatStatusChangeCnt = 0;        //状态变化开始计数
    
    //disable_irq_wake(gBatteryData->dc_det_irq);
    queue_delayed_work(gBatteryData->wq, &gBatteryData->work, 0);
	
    return IRQ_HANDLED;
}

static void rk2918_battery_work(struct work_struct *work)
{
    int ret;
    int irq_flag;
    
    rk28_send_wakeup_key();
    
    free_irq(gBatteryData->dc_det_irq, gBatteryData);
    irq_flag = (!gpio_get_value (gBatteryData->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	ret = request_irq(gBatteryData->dc_det_irq, rk2918_dc_wakeup, irq_flag, "rk2918_battery", gBatteryData);
	if (ret) {
		free_irq(gBatteryData->dc_det_irq, gBatteryData);
	}
	enable_irq_wake(gBatteryData->dc_det_irq);
}

static void rk2918_battery_callback(struct adc_client *client, void *param, int result)
{
    gBatteryData->adc_val = result;
	return;
}

#define POWER_ON_PIN    RK29_PIN4_PA4
static void rk2918_low_battery_check(void)
{
    int i;
    int tmp = 0;
    
    for (i = 0; i < 100; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 100;
    //gBatteryData->adc_val = tmp;
    //gBatVoltage = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
    //rk2918_get_charge_status();
    //gBatCapacity = rk2918_get_bat_capacity_raw(gBatVoltage);
    rk2918_get_charge_status();
    tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
	if(RK29_ADC_CLI_ADD)
		tmp += RK29_ADC_CLI_VALUE;
	else
		tmp -= RK29_ADC_CLI_VALUE;
    gBatCapacity = rk2918_get_bat_capacity_raw(tmp);
    if (gBatCapacity == 0) gBatCapacity = 1;
    printk("rk2918_low_battery_check: gBatVoltage = %d, gBatCapacity = %d\n", gBatVoltage, gBatCapacity);
    
    if (gBatVoltage <= BATT_ZERO_VOL_VALUE + 50)
    {
        printk("low battery: powerdown\n");
        gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
        tmp = 0;
        while(1)
        {
            if(gpio_get_value(POWER_ON_PIN) == GPIO_HIGH)
		    {
			    gpio_set_value(POWER_ON_PIN,GPIO_LOW);
		    }
		    mdelay(5);
		    if (++tmp > 50) break;
		}
    }
    gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
}

static ssize_t rk29_battery_dbg_show(struct class *cls, char *_buf)
{
    return sprintf(_buf, "%d\n", rk29_battery_dbg_level);
}

static ssize_t rk29_battery_dbg_store(struct class *cls, const char *_buf, size_t _count)
{
    rk29_battery_dbg_level = simple_strtoul(_buf, NULL, 16);
    printk("rk29_battery_dbg_level = %d\n",rk29_battery_dbg_level);
    
    return _count;
} 

static struct class *rk29_battery_dbg_class = NULL;
static CLASS_ATTR(rk29_battery_dbg, 0666, rk29_battery_dbg_show, rk29_battery_dbg_store);

static int rk2918_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct adc_client *client;
	struct rk2918_battery_data *data;
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;
	int irq_flag;
	
	printk(KERN_INFO "rk2918_battery: driver initializing start\n");
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}

    //clear io
    data->dc_det_pin     = INVALID_GPIO;
    data->batt_low_pin   = INVALID_GPIO;
    data->charge_set_pin = INVALID_GPIO;
	data->charge_ok_pin  = INVALID_GPIO;
	
	if (pdata && pdata->io_init) {
		ret = pdata->io_init();
		if (ret) 
			goto err_free_gpio1;		
	}
	
	//dc det
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
	#ifndef DC_DET_WITH_USB_INT
    	ret = gpio_request(pdata->dc_det_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto err_free_gpio1;
    	}
	#endif
    	ret = gpio_direction_input(pdata->dc_det_pin);
    	if (ret) {
    		printk("failed to set gpio dc_det input\n");
    		goto err_free_gpio1;
    	}
		
		//gpio_pull_updown(pdata->dc_det_gpio, GPIOPullUp);//important
#if DC_DET_EFFECTIVE	
	    gpio_pull_updown(pdata->dc_det_pin, PullDisable);
#else
	    gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);
#endif
    	data->dc_det_pin   = pdata->dc_det_pin;
    	data->dc_det_level = pdata->dc_det_level;
    }

	//charge set for usb charge
	if (pdata->charge_set_pin != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->charge_set_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto err_free_gpio1;
    	}
    	data->charge_set_pin = pdata->charge_set_pin;
    	data->charge_set_level = pdata->charge_set_level;
    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
    }

	//current control gpio
#if defined(DC_CURRENT_IN_TWO_MODE)
	ret = gpio_request(GPIO_CURRENT_CONTROL, "DC_CURRENT_CONTROL");
	if (ret < 0) {
		pr_err("rk29_battery: failed to request GPIO %d,"
				" error %d\n", GPIO_CURRENT_CONTROL, ret);
		goto err_free_current_control_gpio;
	}

	ret = gpio_direction_output(GPIO_CURRENT_CONTROL, 1);
	if (ret < 0) {
		pr_err("rk29_battery: failed to configure input"
				" direction for GPIO %d, error %d\n",
				GPIO_CURRENT_CONTROL, ret);
		gpio_free(GPIO_CURRENT_CONTROL);
		goto err_free_current_control_gpio;
	}
    gpio_pull_updown(GPIO_CURRENT_CONTROL, 1);
	gpio_set_value(GPIO_CURRENT_CONTROL, 1);
#endif

	//charge_ok
	if (pdata->charge_ok_pin != INVALID_GPIO)
	{
        ret = gpio_request(pdata->charge_ok_pin, NULL);
    	if (ret) {
    		printk("failed to request charge_ok gpio\n");
    		goto err_free_gpio2;
    	}
	
    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->charge_ok_pin);
    	if (ret) {
    		printk("failed to set gpio charge_ok input\n");
    		goto err_free_gpio2;
    	}
    	data->charge_ok_pin   = pdata->charge_ok_pin;
    	data->charge_ok_level = pdata->charge_ok_level;
    }
    
	client = adc_register(0, rk2918_battery_callback, NULL);
    if(!client)
		goto err_adc_register_failed;
    
	memset(gBatVoltageSamples, 0, sizeof(gBatVoltageSamples));
	spin_lock_init(&data->lock);
    data->adc_val = adc_sync_read(client);
	data->client = client;
    data->battery.properties = rk2918_battery_props;
	data->battery.num_properties = ARRAY_SIZE(rk2918_battery_props);
	data->battery.get_property = rk2918_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	data->adc_bat_divider = 414;
	data->bat_max = BATT_MAX_VOL_VALUE;
	data->bat_min = BATT_ZERO_VOL_VALUE;
	DBG("bat_min = %d\n",data->bat_min);
	
#ifdef RK29_USB_CHARGE_SUPPORT
	data->usb.properties = rk2918_usb_props;
	data->usb.num_properties = ARRAY_SIZE(rk2918_usb_props);
	data->usb.get_property = rk2918_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
#endif

	data->ac.properties = rk2918_ac_props;
	data->ac.num_properties = ARRAY_SIZE(rk2918_ac_props);
	data->ac.get_property = rk2918_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
	{
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
#if 0
	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
	{
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
	{
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
	platform_set_drvdata(pdev, data);
	
	INIT_WORK(&data->timer_work, rk2918_battery_timer_work);
	gBatteryData = data;
	
//    irq_flag = (pdata->chg_ok_level) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
//	ret = request_irq(gpio_to_irq(pdata->charge_ok_pin), rk2918_battery_interrupt, irq_flag, "rk2918_battery", data);
//	if (ret) {
//		printk("failed to request irq\n");
//		goto err_irq_failed;
//	}

#ifndef DC_DET_WITH_USB_INT
    if (pdata->dc_det_pin != INVALID_GPIO)
    {
        irq_flag = (!gpio_get_value (pdata->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
    	ret = request_irq(gpio_to_irq(pdata->dc_det_pin), rk2918_dc_wakeup, irq_flag, "rk2918_battery", data);
    	if (ret) {
    		printk("failed to request dc det irq\n");
    		goto err_dcirq_failed;
    	}
    	data->dc_det_irq = gpio_to_irq(pdata->dc_det_pin);
    	//data->wq = create_rt_workqueue("rk2918_battery");
	data->wq = create_workqueue("rk2918_battery");
    	INIT_DELAYED_WORK(&data->work, rk2918_battery_work);
    	
    	enable_irq_wake(gpio_to_irq(pdata->dc_det_pin));
    }
#endif

	setup_timer(&data->timer, rk2918_batscan_timer, (unsigned long)data);
	//data->timer.expires  = jiffies + 2500;
	//add_timer(&data->timer);

	printk("%8x   %8x  %8x  %8x\n\n\n\n\n\n\n ",data,gBatteryData,&data->timer,&gBatteryData->timer);
    
    rk2918_low_battery_check();
    
    rk29_battery_dbg_class = class_create(THIS_MODULE, "rk29_battery");
	class_create_file(rk29_battery_dbg_class,&class_attr_rk29_battery_dbg);
     ret = device_create_file(&pdev->dev, &dev_attr_startget);
     if (ret)
     {
	
		printk("make a mistake in creating devices  attr file\n\n ");
     }
	printk(KERN_INFO "rk2918_battery: driver initialized\n");
	
	return 0;
	
err_dcirq_failed:
    free_irq(gpio_to_irq(pdata->dc_det_pin), data);
    
err_irq_failed:
	free_irq(gpio_to_irq(pdata->charge_ok_pin), data);
    
err_battery_failed:
//	power_supply_unregister(&data->usb);
//err_usb_failed:
err_ac_failed:
	power_supply_unregister(&data->ac);
	
err_adc_register_failed:
err_free_gpio2:
	gpio_free(pdata->charge_ok_pin);
	
err_free_current_control_gpio:
#if defined(DC_CURRENT_IN_TWO_MODE)
	gpio_free(GPIO_CURRENT_CONTROL);
#endif
err_free_gpio1:
    gpio_free(pdata->dc_det_pin);
    
err_data_alloc_failed:
	kfree(data);

    printk("rk2918_battery: error!\n");
    
	return ret;
}

static int rk2918_battery_remove(struct platform_device *pdev)
{
	struct rk2918_battery_data *data = platform_get_drvdata(pdev);
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;
	
	power_supply_unregister(&data->battery);
//	power_supply_unregister(&data->usb);
	power_supply_unregister(&data->ac);
	free_irq(data->irq, data);
	gpio_free(pdata->charge_ok_pin);
	gpio_free(pdata->dc_det_pin);
	kfree(data);
	gBatteryData = NULL;
	return 0;
}

static struct platform_driver rk2918_battery_driver = {
	.probe		= rk2918_battery_probe,
	.remove		= rk2918_battery_remove,
	.suspend	= rk2918_battery_suspend,
	.resume		= rk2918_battery_resume,
	.driver = {
		.name = "rk29_battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk2918_battery_init(void)
{
	return platform_driver_register(&rk2918_battery_driver);
}

static void __exit rk2918_battery_exit(void)
{
	platform_driver_unregister(&rk2918_battery_driver);
}

module_init(rk2918_battery_init);
module_exit(rk2918_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk2918");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");

