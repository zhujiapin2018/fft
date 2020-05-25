#include "lcd.h"
#include "font.h"
//#include "code.h"
//#include "tjpgd.h"
// #include "decode_image.h"


uint8_t font_index[]={"℃阿爱安庵鞍岸昂敖澳八巴坝鲅霸灞白百柏拜班坂版伴蚌包雹宝保堡暴陂碑北贝碚本比毕碧壁濞璧边宾彬滨冰兵秉波播伯泊勃亳博布步部埠彩蔡仓苍沧藏曹草册策岑曾茶查察岔柴禅瀍昌常厂场巢朝潮车郴尘辰陈称成呈承城澄池茌赤冲充崇畴滁楚川船春淳慈磁次枞从丛翠村措错达大傣代岱带戴丹单郸儋当党砀宕刀岛道稻得德的灯登等邓磴滴迪坻底地棣点电甸店垫淀迭叠丁顶鼎定东冻侗峒洞都斗独杜度渡端堆敦多掇峨额鄂恩儿尔洱二法番烦樊繁范方邡坊防房放妃肥费分芬汾丰风封峰烽锋凤奉佛扶芙孚罘浮符涪福抚府附阜复富伽嘎噶改盖甘感干赣冈刚钢岗港皋高藁戈革阁格葛个各根耿工弓公功宫恭巩拱珙共贡沟沽姑古谷鼓固故瓜拐关观官莞馆管冠灌光广归贵桂郭国果哈海邗邯含涵韩寒罕汉行杭濠好浩禾合和河荷菏贺赫鹤黑亨恒横衡红宏虹洪侯后呼胡壶葫湖虎互祜鄠花华骅滑化桦怀淮槐环桓皇黄凰湟煌潢晃珲晖辉徽回汇会惠浑获霍鸡积基及吉级极即集济绩蓟暨稷冀加佳迦家嘉夹郏贾架尖间监犍简碱建剑涧箭江将姜疆绛交郊胶椒蛟焦蕉礁觉揭街节结解介界金津锦进晋缙京泾经荆旌精井景陉竞靖静镜鸠九久酒旧居莒巨句飓卷鄄军君筠峻浚喀卡开凯坎康考柯科可岢克垦崆口库宽矿奎坤昆拉喇腊来莱崃涞赉兰岚蓝澜郎阆琅廊朗浪蒗崂老佬乐勒雷耒垒类棱楞冷离梨犁黎蠡礼李里理鲤澧醴力历立丽利荔栗傈溧连莲涟联廉濂良凉梁两辽聊列烈邻林临麟蔺灵凌陵零岭令浏留流柳六龙隆陇娄楼卢芦庐炉泸鲁陆鹿渌禄碌路潞麓峦栾滦仑伦轮罗萝洛漯吕旅绿略麻马玛霾迈麦满曼芒茫毛茅茂眉梅湄美门们蒙盟勐孟梦弥米汨泌密绵棉勉冕渑苗民岷闵闽敏名明鸣谟磨末莫漠墨默牟谋牡木仫沐牧穆那纳乃奈南囊淖讷内嫩尼年碾聂宁牛农怒诺瓯牌潘攀盘磐沛彭蓬邳皮郫偏票平坪凭屏萍坡颇鄱仆莆蒲濮埔浦普谱七栖齐祁岐其奇耆淇綦旗蕲麒杞启起恰千阡迁铅谦前乾潜黔羌强桥硚谯巧茄且钦秦勤沁青清晴庆邛琼丘邱区朐渠衢曲圈权全泉劝确群壤让饶热仁任日荣容蓉榕融柔如汝乳芮瑞润若撒萨塞赛三桑色沙莎厦山陕汕善鄯商上尚韶邵绍畲社射涉申莘深什神沈审省圣胜嵊师狮施浉十石始氏市手首寿舒疏熟蜀曙沭树墅双水顺朔硕思斯四寺泗松淞嵩苏肃僳睢濉绥随遂穗孙索塔台太泰滩坛郯覃潭汤唐堂棠塘洮桃陶特腾滕藤提天田调铁汀亭通同桐铜潼头突图徒涂土吐团屯托拖脱陀洼瓦佤外湾宛万汪王旺望威微巍为圩围维潍伟尾卫未尉渭蔚魏温文闻汶翁瓮涡沃卧斡乌巫无芜吾吴梧五伍武舞务悟婺雾西昔息浠淅锡溪歙习隰喜细峡霞下夏仙鲜贤咸县献乡芗相香厢湘襄镶祥翔响向项象萧猇霄小孝谢心辛忻新信星邢兴杏雄休修宿秀岫盱徐许叙溆宣玄薛穴雪旬寻浔循逊鸭牙琊崖涯雅亚烟焉鄢延岩炎沿研盐阎兖郾偃砚彦晏堰雁央扬羊阳杨洋漾尧姚遥瑶要耀掖冶野业叶邺伊依猗黟仪夷沂宜眙彝弋义仡邑易峄驿益谊翼阴荫音殷银鄞印英鹰迎荥盈营蓥颍应邕雍永埇攸尤邮犹油游友有酉右于余盂鱼禺隅渝榆虞舆屿宇雨禹玉郁峪域裕豫元园沅垣袁原源远苑月岳越云匀郧运郓蕴杂载赞枣则泽增扎札闸柞寨沾站湛张章彰漳樟长丈招昭召诏赵照肇柘浙贞浈真圳阵振镇征蒸正郑政芝枝知织脂植芷指至志治峙陟中忠钟仲重舟州周洲珠株诸竹主助驻柱祝庄壮准涿卓孜资淄滋子秭梓紫自宗邹足族嘴遵左作"};
uint8_t asc_index[]={" ~`!@#$%^&*()_+-=,./<>?';:\"[]{}\\|1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"};
//  (0) ~(1) `(2) !(3) @(4) #(5) $(6) %(7) ^(8) &(9) *(10) ((11) )(12) _(13) +(14) -(15)
// =(16) ,(17) .(18) /(19) <(20) >(21) ?(22) '(23) ;(24) :(25) "(26) [(27) ](28) {(29) }(30) \(31)
// |(32) 1(33) 2(34) 3(35) 4(36) 5(37) 6(38) 7(39) 8(40) 9(41) 0(42) a(43) b(44) c(45) d(46) e(47)
// f(48) g(49) h(50) i(51) j(52) k(53) l(54) m(55) n(56) o(57) p(58) q(59) r(60) s(61) t(62) u(63)
// v(64) w(65) x(66) y(67) z(68) A(69) B(70) C(71) D(72) E(73) F(74) G(75) H(76) I(77) J(78) K(79)
// L(80) M(81) N(82) O(83) P(84) Q(85) R(86) S(87) T(88) U(89) V(90) W(91) X(92) Y(93) Z(94)
_lcd_dev lcddev;

uint16_t BACK_COLOR, POINT_COLOR;   //背景色，画笔色


//extern spi_device_handle_t spi;
spi_device_handle_t spi;
//uint16_t SPI_LCD_RAM[320*240];//显示缓存
//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_cmd(const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_data(const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
void lcd_data16(const uint16_t *data,int len)
{
	esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*16;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.	
}
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void LCD_WR_DATA8(const uint8_t da) //发送数据-8位参数
{
	lcd_data(&da, 1);
} 

//#define LCD_WR_DATA8 LCD_WR_DATA
void LCD_WR_DATA(uint16_t da)
{
	uint8_t dat[2];
	dat[0] = da >>8;
	dat[1] = da;
	lcd_data(dat, 2);
}	

void LCD_WR_REG(uint8_t da)	 
{	
	lcd_cmd(da);
}

#define BUF_LEN 512

void LCD_Fast_WR_DATA(uint8_t* Color ,uint32_t len)
{
	uint32_t j,k;
	uint8_t *SPI_LCD_RAM = (uint8_t *)malloc(len);
	k = len / BUF_LEN;
	for(j = 0; j < k ;j++)//刷新一行数据
		lcd_data(Color + BUF_LEN*j, BUF_LEN);	

	k = len % BUF_LEN;
	if(k)
		lcd_data(SPI_LCD_RAM + BUF_LEN*j, k);//最后写入不满32字节的包
	free(SPI_LCD_RAM);	 
}  

void LCD_Fast_WR_DATA16(uint16_t* Color ,uint32_t len)
{
	uint32_t i,k;
	uint8_t *SPI_LCD_RAM = (uint8_t *)malloc(2 * len);	
	printf("alloc memory ok\n");
	k = len*2 / BUF_LEN;
	for (i = 0;i < len; i++)
	{
		SPI_LCD_RAM[ 2*i ] = Color[i] >> 8;//高位
		SPI_LCD_RAM[ 2*i + 1 ] = Color[i]; //低位
	}
	printf("set spi ram\n");
	for(i = 0; i < k ;i++)//刷新一行数据
		lcd_data(SPI_LCD_RAM + BUF_LEN*i, BUF_LEN);

	k = len*2 % BUF_LEN;
	if(k)	 
		lcd_data(SPI_LCD_RAM + BUF_LEN*i, k);//最后写入不满32字节的包
	printf("lcd data ok \n");
	free(SPI_LCD_RAM);		 
}  

void LCD_Fast_WR_Color_DATA16(uint16_t Color ,uint32_t len)
{
	uint32_t i,k;
	uint8_t *SPI_LCD_RAM =NULL;
	SPI_LCD_RAM = (uint8_t *)malloc( BUF_LEN);	
	if(SPI_LCD_RAM==NULL)
		printf("malloc failed!");
	k = len*2 / BUF_LEN;
	for (i = 0;i < BUF_LEN/2; i++)
	{
		SPI_LCD_RAM[ 2*i ] = Color >> 8;//高位
		SPI_LCD_RAM[ 2*i + 1 ] = Color; //低位
	}	

	for(i = 0; i < k ;i++)//刷新一行数据
		lcd_data(SPI_LCD_RAM , BUF_LEN);
	k = len*2 % BUF_LEN;
	if(k)	 
		lcd_data(SPI_LCD_RAM, k);//最后写入不满32字节的包

	free(SPI_LCD_RAM);
	/*
	uint32_t i,k;
	uint8_t *SPI_LCD_RAM =NULL;
	SPI_LCD_RAM = (uint8_t *)malloc(2 * len);	
	if(SPI_LCD_RAM==NULL)
		printf("malloc failed!");
	k = len*2 / BUF_LEN;
	for (i = 0;i < len; i++)
	{
		SPI_LCD_RAM[ 2*i ] = Color >> 8;//高位
		SPI_LCD_RAM[ 2*i + 1 ] = Color; //低位
	}	

	for(i = 0; i < k ;i++)//刷新一行数据
		lcd_data(SPI_LCD_RAM + BUF_LEN*i, BUF_LEN-1);
	k = len*2 % BUF_LEN;
	if(k)	 
		lcd_data(SPI_LCD_RAM + BUF_LEN*i, k);//最后写入不满32字节的包

	free(SPI_LCD_RAM);//*/
} 

void LCD_WR_REG_DATA(uint16_t reg,uint16_t da)
{
    LCD_WR_REG(reg);
	LCD_WR_DATA(da);
}
//读LCD数据
uint16_t LCD_RD_DATA(void)
{	
	return 0;
}
//读寄存器数据
uint16_t LCD_ReadReg(uint16_t LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);
	return LCD_RD_DATA();
}

//开始写GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);  
}

//LCD写GRAM
void LCD_WriteRAM(uint16_t RGB_Code)
{
	LCD_WR_DATA(RGB_Code);  
}


//从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
//通过该函数转换
//c:GBR格式的颜色值
//返回值：RGB格式的颜色值
uint16_t LCD_BGR2RGB(uint16_t c)
{
	uint16_t  r,g,b,rgb;   
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	 
	rgb=(b<<11)+(g<<5)+(r<<0);		 
	return(rgb);
} 

void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
	LCD_WR_REG(0x2a);
	LCD_WR_DATA(x1);
	LCD_WR_DATA(x2);
	
	LCD_WR_REG(0x2b);
	LCD_WR_DATA(y1);
	LCD_WR_DATA(y2);

	LCD_WR_REG(0x2C);					 						 
}
//设置光标位置
//Xpos:横坐标
//Ypos:纵坐标
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
    LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA(Xpos);	 
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA(Ypos);
} 

//设置LCD的自动扫描方向
//注意:其他函数可能会受到此函数设置的影响(尤其是9341/6804这两个奇葩),
//所以,一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
//dir:0~7,代表8个方向(具体定义见lcd.h)
//9320/9325/9328/4531/4535/1505/b505/8989/5408/9341/5310/5510等IC已经实际测试	   	   
void LCD_Scan_Dir(uint8_t dir)
{
	uint16_t regval=0;
	//uint16_t dirreg=0;
	uint16_t temp;  
	switch(dir)
	{
		case L2R_U2D://从左到右,从上到下
			regval|=(0<<7)|(0<<6)|(0<<5)|(1<<4); 
			break;
		case L2R_D2U://从左到右,从下到上
			regval|=(1<<7)|(0<<6)|(0<<5)|(1<<4); 
			break;
		case R2L_U2D://从右到左,从上到下
			regval|=(0<<7)|(1<<6)|(0<<5)|(1<<4); 
			break;
		case R2L_D2U://从右到左,从下到上
			regval|=(1<<7)|(1<<6)|(0<<5)|(1<<4); 
			break;	

		case U2D_L2R://从上到下,从左到右
			regval|=(0<<7)|(0<<6)|(1<<5)|(0<<4); 
			break;
		case U2D_R2L://从上到下,从右到左
			regval|=(0<<7)|(1<<6)|(1<<5)|(0<<4); 
			break;
		case D2U_L2R://从下到上,从左到右
			regval|=(1<<7)|(0<<6)|(1<<5)|(0<<4); 
			break;
		case D2U_R2L://从下到上,从右到左
			regval|=(1<<7)|(1<<6)|(1<<5)|(0<<4); 
			break;
  	}
	//LCD_WR_REG_DATA(dirreg,regval);
	LCD_WR_REG(0x36); 
	LCD_WR_DATA8(regval|0x08);
	if((regval&0X20)||lcddev.dir==1)
	{
		if(lcddev.width<lcddev.height)//交换X,Y
		{
			temp=lcddev.width;
			lcddev.width=lcddev.height;
			lcddev.height=temp;
		}
	}else  
	{
		if(lcddev.width>lcddev.height)//交换X,Y
		{
			temp=lcddev.width;
			lcddev.width=lcddev.height;
			lcddev.height=temp;
		}
	}  
}   

void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	//LCD_SetCursor(x,y);		//设置光标位置 
	//LCD_WriteRAM_Prepare();	//开始写入GRAM
	Address_set(x,y,x,y);
	LCD_WR_DATA(POINT_COLOR); 
}

//快速画点
//x,y:坐标
//color:颜色
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA(x);	 
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA(y);
	
	LCD_WR_REG_DATA(lcddev.wramcmd,color);		
}	

//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LCD_Display_Dir(uint8_t dir)
{
	if(dir < 4)			//竖屏
	{
		lcddev.dir=0;	//竖屏
		lcddev.width=240;
		lcddev.height=320;
		
		lcddev.wramcmd=0X2C;
		lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;  	 
		
	}else 				//横屏
	{	  				
		lcddev.dir=1;	//横屏
		lcddev.width=320;
		lcddev.height=240;
		
		lcddev.wramcmd=0X2C;
		lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;  	 		
		
	} 
	LCD_Scan_Dir(dir);	//默认扫描方向
}

//设置窗口,并自动设置画点坐标到窗口左上角(sx,sy).
//sx,sy:窗口起始坐标(左上角)
//width,height:窗口宽度和高度,必须大于0!!
//窗体大小:width*height.
//68042,横屏时不支持窗口设置!! 
void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{    
	width=sx+width-1;
	height=sy+height-1;
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA(sx);	 
	LCD_WR_DATA(width);   
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA(sy); 
	LCD_WR_DATA(height); 
} 
//读取个某点的颜色值	 
//x,y:坐标
//返回值:此点的颜色
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
 	uint16_t r=0,g=0,b=0;
	uint16_t LCD_RAM;
	if(x>=lcddev.width||y>=lcddev.height)return 0;	//超过了范围,直接返回		   
	LCD_SetCursor(x,y);	    
	LCD_WR_REG(0X2E);//9341/6804/3510 发送读GRAM指令

	LCD_RAM = LCD_RD_DATA();//第一次为假读
	LCD_RAM = LCD_RD_DATA();
	printf("point date4:0x%x\n",LCD_RAM);   

	if(LCD_RAM)r=0;							//dummy Read	     
 	r=LCD_RAM;  		  						//实际坐标颜色
 		  
	b = LCD_RAM; 
	g = r&0XFF;		//对于9341/5310/5510,第一次读取的是RG的值,R在前,G在后,各占8位
	g <<= 8;
	
	return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));
}	

//LCD开启显示
void LCD_DisplayOn(void)
{	
	LCD_WR_REG(0x29);
}	 
//LCD关闭显示
void LCD_DisplayOff(void)
{	 
	LCD_WR_REG(0x28);
}  

void LCD_delay(int t)
{
	vTaskDelay(t / portTICK_RATE_MS);
}


/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

extern spi_device_handle_t tp_spi;

void Lcd_Init(void)
{  
	esp_err_t ret; 
	gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    // gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_level(PIN_NUM_DC, 1);
    // gpio_set_level(PIN_NUM_RST, 1);
	gpio_set_level(PIN_NUM_BCKL,1);

    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
		//.max_transfer_sz = 320 * 240 * 2,
    }; 
    spi_device_interface_config_t devcfg={
		.clock_speed_hz = 32000000,				// Initial clock out at 8 MHz
		.mode = 0,								// SPI mode 0
		.spics_io_num = PIN_NUM_CS,				// set SPI CS pin
        .queue_size=BUF_LEN,                    //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
	printf("Initialize the SPI bus OK\n");
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
	printf("Attach the LCD to the SPI bus OK\n");

	// spi_device_interface_config_t devcfg_1={
	// 	.clock_speed_hz = 4000000,				// Initial clock out at 8 MHz
	// 	.mode = 3,								// SPI mode 0
	// 	.spics_io_num = PIN_NUM_TCS,			// set SPI CS pin
    //     .queue_size=7,                    //We want to be able to queue 7 transactions at a time
    // };
    // //Attach the touch to the SPI bus
    // ret=spi_bus_add_device(HSPI_HOST, &devcfg_1, &tp_spi);
    // assert(ret==ESP_OK);
	
	// mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    // mcpwm_config_t pwm_config;
    // pwm_config.frequency = 1000;    //frequency = 500Hz,
    // pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    // //pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    // pwm_config.counter_mode = MCPWM_UP_COUNTER;
    // pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	// brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 80.0);
	
	//LCD_BL_0;

    // LCD_REST_0;
	// LCD_delay(100);
	// LCD_REST_1;
	// LCD_delay(200);

    lcddev.dir = 0;	//竖屏
    lcddev.width = 240;
    lcddev.height = 240;
    lcddev.id = 0x7789;
    lcddev.wramcmd = 0X2C;
    lcddev.setxcmd = 0X2A;
    lcddev.setycmd = 0X2B;

    LCD_WR_REG(0xCF);  
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0XC1); 
    LCD_WR_DATA8(0X30); 

    LCD_WR_REG(0xED);  
    LCD_WR_DATA8(0x64); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0X12); 
    LCD_WR_DATA8(0X81); 

    LCD_WR_REG(0xE8);  
    LCD_WR_DATA8(0x85); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x78); 

    LCD_WR_REG(0xCB);  
    LCD_WR_DATA8(0x39); 
    LCD_WR_DATA8(0x2C); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x34); 
    LCD_WR_DATA8(0x02); 
    
    LCD_WR_REG(0xF7);  
    LCD_WR_DATA8(0x20); 

    LCD_WR_REG(0xEA);  
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x00); 
    
    LCD_WR_REG(0xC0);    //Power control 
    LCD_WR_DATA8(0x23);   //VRH[5:0] 

    LCD_WR_REG(0xC1);    //Power control 
    LCD_WR_DATA8(0x10);   //SAP[2:0];BT[3:0] 

    LCD_WR_REG(0xC5);    //VCM control 
    LCD_WR_DATA8(0x3e); //对比度调节
    LCD_WR_DATA8(0x28); 

    LCD_WR_REG(0xC7);    //VCM control2 
    LCD_WR_DATA8(0x86);  //--

    LCD_WR_REG(0x36);    // Memory Access Control 
    LCD_WR_DATA8(0x48); //	   //48 68竖屏//28 E8 横屏

    LCD_WR_REG(0x3A);    
    LCD_WR_DATA8(0x55); 

    LCD_WR_REG(0xB1);    
    LCD_WR_DATA8(0x00);  
    LCD_WR_DATA8(0x18); 

    LCD_WR_REG(0xB6);    // Display Function Control 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x82);
    LCD_WR_DATA8(0x27);  
    
    LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
    LCD_WR_DATA8(0x00); 
    
    LCD_WR_REG(0x26);    //Gamma curve selected 
    LCD_WR_DATA8(0x01); 

    LCD_WR_REG(0xE0);    //Set Gamma 
    LCD_WR_DATA8(0x0F); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0x2B); 
    LCD_WR_DATA8(0x0C); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x4E); 
    LCD_WR_DATA8(0xF1); 
    LCD_WR_DATA8(0x37); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x10); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x09); 
    LCD_WR_DATA8(0x00); 

    LCD_WR_REG(0XE1);    //Set Gamma 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x14); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x11); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0xC1); 
    LCD_WR_DATA8(0x48); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x0F); 
    LCD_WR_DATA8(0x0C); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0x36); 
    LCD_WR_DATA8(0x0F); 

    LCD_WR_REG(0x2B); 
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x3f);
    
    LCD_WR_REG(0x2A); 
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xef);

    LCD_WR_REG(0x11);    //Exit Sleep 
    
    LCD_delay(120); 

    LCD_WR_REG(0x29);    //Display on 
    LCD_WR_REG(0x2c);
    //LCD_BL_1;
	LCD_Display_Dir(R2L_U2D);
	//LCD_Display_Dir(U2D_L2R);
	LCD_Clear(WHITE);
}
//清屏函数
//Color:要清屏的填充色

void LCD_Clear(uint16_t Color)
{
	Address_set(0,0,lcddev.width - 1,lcddev.height - 1);
	LCD_Fast_WR_Color_DATA16(Color,(lcddev.width)*(lcddev.height)); 	
}
 
//画一个大点
//POINT_COLOR:此点的颜色
void LCD_DrawPoint_big(uint16_t x,uint16_t y)
{
	LCD_Fill(x-1,y-1,x+1,y+1,POINT_COLOR);
} 
//在指定区域内填充指定颜色
//区域大小:
//  (xend-xsta)*(yend-ysta)
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{   
	Address_set(xsta,ysta,xend,yend);      //设置光标位置 
	LCD_Fast_WR_Color_DATA16(color,(xend-xsta+1) * (yend-ysta+1));    				  	    
} 

//在指定区域内填充指定颜色块			 
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Color_Fill8(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint8_t *color)
{  
	Address_set(sx,sy,ex,ey);
	LCD_Fast_WR_DATA(color,2*(ex-sx+1) * (ey-sy+1));
}
//在指定区域内填充指定颜色块			 
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
{  
	Address_set(sx,sy,ex,ey);
	LCD_Fast_WR_DATA16(color,(ex-sx+1) * (ey-sy+1));	
}  
//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}
//画矩形
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_Fill(x1,y1,x2,y1,POINT_COLOR);
	LCD_Fill(x1,y1,x1,y2,POINT_COLOR);
	LCD_Fill(x1,y2,x2,y2,POINT_COLOR);
	LCD_Fill(x2,y1,x2,y2,POINT_COLOR);
}
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void LCD_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  		LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//使用Bresenham算法画圆     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
}
//在指定位置显示一个字符

//num:要显示的字符:" "--->"~"
//mode:叠加方式(1)还是非叠加方式(0)
//在指定位置显示一个字符

//num:要显示的字符:" "--->"~"

//mode:叠加方式(1)还是非叠加方式(0)

void LCD_ShowChar(uint16_t x,uint16_t y,char num,uint8_t size,uint8_t mode)
{  							  
    uint8_t temp = 0,t1,t,xwidth;
	uint16_t y0=y;
	uint16_t colortemp=POINT_COLOR;   
	uint16_t SPI_LCD_RAM[16*16];
	xwidth = size/2;   			     
	//设置窗口		  
	//Address_set(x,y,x+size/2-1,y+size-1); 
	num = num - ' ';//得到偏移后的值
	if(!mode) //非叠加方式
	{
	    for(t=0;t < size;t++)
	    {   
			if(size == 12)temp=asc2_1206[(uint8_t)num][t];  //调用1206字体
			else temp=asc2_1608[(uint8_t)num][t];		 //调用1608字体 	                          
	        for(t1 = 0;t1 < 8;t1++)
			{			    
		        if(temp&0x01)POINT_COLOR = colortemp;
				else POINT_COLOR = BACK_COLOR;
				SPI_LCD_RAM[t*xwidth + t1] = POINT_COLOR;	//先存起来再显示
				temp>>=1;
				y++;
				if(y>=lcddev.height){POINT_COLOR=colortemp;return;}//超区域了
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//超区域了
					break;
				}
			}  	 
	    }    
	}else//叠加方式
	{
	    for(t=0;t<size;t++)
	    {   
			if(size == 12)temp=asc2_1206[(uint8_t)num][t];  //调用1206字体
			else temp=asc2_1608[(uint8_t)num][t];		 //调用1608字体 	                          
	        for(t1=0;t1 < 8;t1++)
			{			    
		        if(temp&0x01)//从左到右 逐列扫描
				{
					//LCD_DrawPoint(x,y); 
					SPI_LCD_RAM[t*xwidth + t1] = POINT_COLOR;	//先存起来再显示
				}
				else SPI_LCD_RAM[t*xwidth + t1] = BACK_COLOR;
				
				temp>>=1;
				y++;
				if(y>=lcddev.height){POINT_COLOR=colortemp;return;}//超区域了
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//超区域了
					break;
				}
			}  	 
	    }     
	}
	LCD_Color_Fill(x - size/2,y0,x-1,y+size-1,SPI_LCD_RAM);
	POINT_COLOR=colortemp;	    	   	 	  
} 

//m^n函数
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//color:颜色
//num:数值(0~4294967295);	
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	num=(uint16_t)num;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+8*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+8*t,y,temp+48,size,0); 
	}
} 

/*********************************************************************
函数名：	PrintHZ16
功能：		在显示器上显示汉字字符串点阵，
参数：		x，y：预显示字符串的位置，左上角
			c：字符串，汉字字符串，仅unsigned short * 类型
			len：字符串长度，汉字个数
			convert：显示方式，0--正常显示，1--反色显示
返回至：	void
*********************************************************************/
// void PrintHZ16(int x,int y,unsigned short *c,int len,unsigned char convert)
// {
// 	setfillstyle(EMPTY_FILL,0);
// 	bar(x,y,x+len*16,y+16);


// 	unsigned int c1,c2;
// 	int i1,i2,i3,rec;
// 	long l;
// 	char by[32];
// 	for(int i=0;i<len;i++)
// 	{
// 		 c2=((c[i]>>8)-0xa1)&0x7f;
// 		 c1=((c[i]&0xff)-0xa1)&0x7f;
// 		 rec=c1*94+c2;
// 		 l=rec*32L;
// 		 fseek(handle,l,SEEK_SET);
// 		 fread(by,32,1,handle);
// 		 for(i1=0;i1<16;i1++)
// 		 for(i2=0;i2<2;i2++)
// 		 for(i3=0;i3<8;i3++)
// 		 if(getbit(by[i1*2+i2],7-i3))
// 		 {
// 			if(convert==0)
// 				putpixel(x+i2*8+i3,y+i1,15);
// 		 }
// 		 else
// 		 {
// 			if(convert==1)
// 				putpixel(x+i2*8+i3,y+i1,15);
// 		 }
// 		 x=x+16;

//       }
// }

//显示中英文混合字符串，16×16,输入字符中文为utf8编码，每个字占用3字节
void LCD_ShowString2(uint16_t x,uint16_t y,char *p)
{
	int offset;
	int len=0;//
	for(int c1=0;c1<strlen(p);c1++)
	{
		unsigned char ch=(unsigned char)p[c1];
		if(ch&0x80)
		{
			c1+=3;
			len++;
		}
		else
		{
			len++;
		}
	}
	unsigned char key[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
	uint16_t *SPI_LCD_RAM=(uint16_t*)malloc(16*16*len*sizeof(uint16_t));
//	uint16_t SPI_LCD_RAM[256]={0};
	for(int c=0;c<len;c++)
	{
		
		offset=-1;
		for(int c2=0;c2<sizeof(font_index)/sizeof(uint8_t)/3;c2++)
		{
			if(memcmp(p+c*3,font_index+c2*3,sizeof(uint8_t)*3)==0)
			{
				offset=c2*3*32;
			}
		}
		if(offset==-1)
		{
			printf("cannot find string in font index\n");
			continue;
		}
//		offset = (94*(unsigned int)((p[c*2]-0xa0-1)&0x7f)+((p[c*2+1]-0xa0-1)&0x7f))*32;
//		printf("offset=%d\n",offset);
		// if(offset>hzk16s_fnt_end-hzk16s_fnt_start)
		// {
		// 	free(SPI_LCD_RAM);
		// 	continue;
		// }
		for(int k=0; k<16; k++)//行
		{
        	for(int j=0; j<2; j++)//一行16个点，分两部分
			{
            	for(int i=0; i<8; i++)//每个字节表示8个点
				{
                	uint8_t flag = hzks_fon_start[k*2+j+offset]&key[i];
//					printf(flag?"#":" ");
//					SPI_LCD_RAM[0]=1;
//					printf("i=%d,j=%d,k=%d\n",i,j,k);
					SPI_LCD_RAM[i+j*8+(k*len+c)*16]=flag?POINT_COLOR:BACK_COLOR;
            	}
        	}
//			printf("\n");
    	}
	}
	LCD_Color_Fill(x,y,x + 16*len - 1,y + 16 -1,SPI_LCD_RAM);

}
/*
//显示ascii字符·32×16
void LCD_ShowAscii(uint16_t x,uint16_t y,char *p)
{
	int offset;
	int len=strlen(p);
	unsigned char key[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
	uint16_t *SPI_LCD_RAM=(uint16_t*)malloc(32*16*len*sizeof(uint16_t));
	for(int c=0;c<len;c++)
	{
		offset=p[c];
		printf("offset=%d:\"%c\"\n",p[c],p[c]);
		for(int k=0; k<32; k++)//行
		{
        	for(int j=0; j<2; j++)//一行16个点，分2部分
			{
            	for(int i=0; i<8; i++)//每个字节表示8个点
				{
                	uint8_t flag = asc_fon_start[k*2+j+offset]&key[i];
//					printf(flag?"#":" ");
					SPI_LCD_RAM[i+j*8+(k*len+c)*16]=flag?POINT_COLOR:BACK_COLOR;
            	}
        	}
//			printf("\n");
    	}
	}
	LCD_Color_Fill(x,y,x + 16*len - 1,y + 32 -1,SPI_LCD_RAM);
}*/

void LCD_ShowHanzi(uint16_t x,uint16_t y,char *p)
{
	int offset;
	int pos=0;
	int len=0;//strlen(p)/3;//utf8编码，每个汉字占用3个字节
	for(int a=0;a<strlen(p);a++)
	{
		if(p[a]==0)
			break;
		if(p[a]&0x80)
		{
//			printf("p[a]=%02X %02X %02X\n",p[a],p[a+1],p[a+2]);
			a+=2;
			len++;
		}
		len++;
	}
//	printf("len=%d\n",len);
	unsigned char key[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
	uint16_t *SPI_LCD_RAM=(uint16_t*)malloc(32*16*len*sizeof(uint16_t));

	for(int c=0;c<strlen(p);c++)
	{
		offset=-1;
		if(p[c]&0x80)//汉字
		{
			for(int c2=0;c2<sizeof(font_index)/sizeof(uint8_t)/3;c2++)
			{
	//			printf("c2=%d\n",c2);
//				if(memcmp(p+c*3,font_index+c2*3,sizeof(uint8_t)*3)==0)
				if(p[c]==font_index[c2*3])
				{
					if(p[c+1]==font_index[c2*3+1]&& p[c+2]==font_index[c2*3+2])
					{
						offset=c2*128;
//						printf("find string at pos %d\n",c2);
						break;
					}
				}
			}
			if(offset==-1)
			{
//				printf("cannot find string in font index\n");
				c+=2;
				pos+=2;
				continue;
			}
	//		offset = (94*(unsigned int)((p[c*2]-0xa0-1)&0x7f)+((p[c*2+1]-0xa0-1)&0x7f))*32;
	//		printf("offset=%d\n",offset);
			// if(offset>hzk16s_fnt_end-hzk16s_fnt_start)
			// {
			// 	free(SPI_LCD_RAM);
			// 	continue;
			// }
			for(int k=0; k<32; k++)//行
			{
				for(int j=0; j<4; j++)//一行32个点，分4部分
				{
					for(int i=0; i<8; i++)//每个字节表示8个点
					{
						uint8_t flag = hzks_fon_start[k*4+j+offset]&key[i];
//						printf(flag?"#":" ");
	//					SPI_LCD_RAM[0]=1;
	//					printf("i=%d,j=%d,k=%d\n",i,j,k);
						SPI_LCD_RAM[i+j*8+(k*len+pos)*16]=flag?POINT_COLOR:BACK_COLOR;
					}
				}
//				printf("\n");
			}
			c+=2;
			pos+=2;

		}
		else
		{
//			printf("p=%c     :",p[c]);
			for(int c2=0;c2<sizeof(asc_index)/sizeof(uint8_t);c2++)
			{
				if(p[c]==asc_index[c2])
				{
					offset=c2;
//					printf("find asc at pos %d\n",c2);
					break;
				}
			}
			if(offset==-1)
			{
//				printf("cannot find string in asc index\n");
				pos++;
				continue;
			}

			for(int k=0; k<32; k++)//行
			{
				for(int j=0; j<2; j++)//一行16个点，分2部分
				{
//					printf("%02X:",asc3216[offset][k*2+j]);
					for(int i=0; i<8; i++)//每个字节表示8个点
					{
						uint8_t flag = asc3216[offset][k*2+j]&key[i];
//						printf(flag?"*":"_");
	//					SPI_LCD_RAM[0]=1;
	//					printf("i=%d,j=%d,k=%d\n",i,j,k);
						SPI_LCD_RAM[i+j*8+(k*len+pos)*16]=flag?POINT_COLOR:BACK_COLOR;
					}
				}
//				printf("\n");
			}
			pos++;
		}
		
	}
	int x2=(240-len*16)/2;
	LCD_Color_Fill(x2,y,x2 + 16*len - 1,y + 32 -1,SPI_LCD_RAM);
	free(SPI_LCD_RAM);
}

//显示2个数字
//x,y:起点坐标
//num:数值(0~99);	 
void LCD_Show2Num(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
	 	LCD_ShowChar(x+8*t,y,temp+'0',size,0); 
	}
} 
// //显示天气图标
// //x,y:起点坐标
// //width,height:宽度，高度
// //code:天气代码
// void LCD_ShowImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,int code)
// {
// 	uint16_t *SPI_LCD_RAM=(uint16_t*)malloc(width*height*sizeof(uint16_t));
// 	uint16_t **pixels;
// 	printf("code=%d\n",code);
// 	esp_err_t ret= decode_image(&pixels,code);///to be modified
// 	if(ret!=ESP_OK)
// 	{
// 		free(SPI_LCD_RAM);
// 		printf("decode jpg failed\n");
// 		return;
// 	}
// 	for(int i=0;i<height;i++)
// 	{
// //		printf("paint line %d",i);
// //		LCD_Color_Fill(x,y+i,width,1,SPI_LCD_RAM[i]);
// //		printf("copy line %d\n",i);
// 		memcpy(SPI_LCD_RAM+width*i,pixels[i],(sizeof(uint16_t))*width);
// 		free(pixels[i]);
// 	}
// 	free(pixels);
// 	LCD_Color_Fill(x,y,x+width-1,y+height-1,SPI_LCD_RAM);
// 	free(SPI_LCD_RAM);
// }
//显示字符串
//x,y:起点坐标
//width,height:区域大小  
//size:字体大小
//*p:字符串起始地址		  
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,char *p)
{         
	uint16_t x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }  
}
//在指定位置显示一个汉字(32*32大小)
//dcolor为内容颜色，gbcolor为背静颜色
void showhanzi(unsigned int x,unsigned int y,char *p,uint8_t size)	
{  
	unsigned char i,j;
	unsigned char *temp = (uint8_t *)p;  
	uint16_t SPI_LCD_RAM[32*32];
	uint16_t cnt = 32 * mypow(size/16,2); 	
	for(j=0;j<cnt;j++)
	{
		for(i=0;i<8;i++)
		{ 		     
		 	if((*temp&(1<<i))!=0)	
				SPI_LCD_RAM[j*8 + i] = POINT_COLOR;
			else
				SPI_LCD_RAM[j*8 + i] = BACK_COLOR; 
		}
		temp++;
	 }
	 LCD_Color_Fill(x,y,x + size - 1,y + size -1,SPI_LCD_RAM);
}

// void showimage(uint16_t x,uint16_t y) //显示40*40图片
// {  
// 	uint16_t i,j,k;
// 	uint16_t da;
// 	uint16_t SPI_LCD_RAM[40*40];
// 	k=0;
// 	for(i=0;i<40;i++)
// 	{	
// 		for(j=0;j<40;j++)
// 		{
// 			da=qqimage[k*2+1];
// 			da<<=8;
// 			da|=qqimage[k*2]; 
// 			SPI_LCD_RAM[i*40 + j] = da;					
// 			k++;  			
// 		}
// 	}
// 	LCD_Color_Fill(x,y,x+39,y+39,SPI_LCD_RAM);
// }
