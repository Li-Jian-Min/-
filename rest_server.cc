/*
   Mathieu Stefani, 07 février 2016

   Example of a REST endpoint with routing
*/

#include <algorithm>

#include <pistache/http.h>
#include <pistache/router.h>
#include <pistache/endpoint.h>

//---------------------------------------------------------------------------
#include <pistache/peer.h>
#include <pistache/http_headers.h>
#include <pistache/cookie.h>
#include <pistache/net.h>
#include <pistache/common.h>



#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include "adc_ti_ads8556_dev.h"
#include <json/json.h>
#include <iostream>
#include <string>

#define  PI     3.1415926535897932384626433832795028841971               //定義圓周率值
//#define  FFT_N    131072                                                    //定義福利葉變換的點數
float the_zero=1.0;

int samplerate = 131072;
int FFT_N=samplerate;
int samplelength=samplerate;
int ad_range,vref,range;
float v_scale=80;
float i_scale=1;
int16_t sync_deta[6*500000]={0};
int sync_rate=0;
int sync_length=0;
struct compx {float real,imag;};                                    //定義一個復數結構

/*	struct compx s1[FFT_N]={0};                                              //FFT輸入和輸出：從S[1]開始存放，根據大小自己定義
	struct compx s2[FFT_N]={0};
	struct compx s3[FFT_N]={0};
	struct compx s4[FFT_N]={0};
	struct compx s5[FFT_N]={0};
	struct compx s6[FFT_N]={0};
*/
float report_buf[23];


//---------------------------------------------------------------------------

using namespace std;
using namespace Pistache;

//----------------------------------------------------------------------------
/*******************************************************************
  函數原型：struct compx EE(struct compx b1,struct compx b2)  
  函數功能：對兩個復數進行乘法運算
  輸入參數：兩個以聯合體定義的復數a,b
  輸出參數：a和b的乘積，以聯合體的形式輸出
 *******************************************************************/
struct compx EE(struct compx a,struct compx b)      
{
	struct compx c;

	c.real=a.real*b.real-a.imag*b.imag;
	c.imag=a.real*b.imag+a.imag*b.real;
	
	return(c);
}
/*****************************************************************
  函數原型：void FFT(struct compx *xin,int N)
  函數功能：對輸入的復數組進行快速傅裏葉變換（FFT）
  輸入參數：*xin復數結構體組的首地址指針，struct型
 *****************************************************************/
void FFT(struct compx *xin)
{
	int f , m, nv2, nm1, i, k, l, j = 0;
	struct compx u,w,t;

	nv2 = FFT_N / 2;                   //變址運算，即把自然順序變成倒位序，采用雷德算法
	nm1 = FFT_N - 1;  
	for(i = 0; i < nm1; i++)        
	{
		if(i < j)                      //如果i<j,即進行變址
		{
			t = xin[j];           
			xin[j] = xin[i];
			xin[i] = t;
		}
		k = nv2;                       //求j的下一個倒位序
		while( k <= j)                 //如果k<=j,表示j的最高位為1   
		{           
			j = j - k;                 //把最高位變成0
			k = k / 2;                 //k/2，比較次高位，依次類推，逐個比較，直到某個位為0
		}
		j = j + k;                     //把0改為1
	}

	{
		int le , lei, ip;                            //FFT運算核，使用蝶形運算完成FFT運算
		
		f = FFT_N;
		for(l = 1; (f=f/2)!=1; l++)                  //計算l的值，即計算蝶形級數
			;
		for( m = 1; m <= l; m++)                         // 控制蝶形結級數
		{                                        //m表示第m級蝶形，l為蝶形級總數l=log（2）N
			le = 2 << (m - 1);                            //le蝶形結距離，即第m級蝶形的蝶形結相距le點
			lei = le / 2;                               //同一蝶形結中參加運算的兩點的距離
			u.real = 1.0;                             //u為蝶形結運算系數，初始值為1
			u.imag = 0.0;
			w.real = cos(PI / lei);                     //w為系數商，即當前系數與前一個系數的商
			w.imag = -sin(PI / lei);
			for(j = 0;j <= lei - 1; j++)                   //控制計算不同種蝶形結，即計算系數不同的蝶形結
			{
				for(i = j; i <= FFT_N - 1; i = i + le)            //控制同一蝶形結運算，即計算系數相同蝶形結
				{
					ip = i + lei;                           //i，ip分別表示參加蝶形運算的兩個節點
					t = EE(xin[ip], u);                    //蝶形運算，詳見公式
					xin[ip].real = xin[i].real - t.real;
					xin[ip].imag = xin[i].imag - t.imag;
					xin[i].real = xin[i].real + t.real;
					xin[i].imag = xin[i].imag + t.imag;
				}
				u = EE(u, w);                           //改變系數，進行下一個蝶形運算
			}
		}
	}
}
/************************************************************
  函數原型：void main() 
  函數功能：測試FFT變換，演示函數使用方法
  輸入參數：無
  輸出參數：無
 ************************************************************/
int setting_samplerate()
{
	int fd;

	printf("open adc device\n");
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		return -1;
	}

	int ret;
	uint32_t val;

	printf("setting samplerate\n");
	val = samplerate;
	ret = ioctl(fd, ADC_IOCTL_SET_SAMPLERATE, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return ret;
	}
	printf("Samplerate is set to %d\n", val);

	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
	}
	return 0;

}

int setting_samplength(int sample_length)
{


	int fd;

	printf("open adc device\n");
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		return -1;
	}

	int ret;
	uint32_t val;

	printf("setting sample length\n");
	val = sample_length;
	ret = ioctl(fd, ADC_IOCTL_SET_LENGTH, &val);
	if (ret == -1) {                 
		printf("ioctl error!\n");
		return -2;
	}
	printf("Length is set to %d\n", val);

	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
		return -3;
	}
	return 0;
}

int set_ad_input_range(){
	
	int fd;

	printf("open adc device\n");
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		return -1;
	}
	int ret;
	uint32_t val;

	printf("setting range\n");
	val = range;
	ret = ioctl(fd, ADC_IOCTL_SET_RANGE, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return -2;
	}
	printf("range is set to %d\n", val);

	printf("setting vref\n");
	val = vref;
	ret = ioctl(fd, ADC_IOCTL_SET_VREF, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return -3;
	}
	printf("vref is set to %d\n", val);
	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
		return -4;
	}

	return 0;
}


int initial_ads8556()
{
	int fd;

	printf("open adc device\n");
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		return -1;
	}

	int ret;
	uint32_t val;

	printf("setting samplerate\n");
	val = samplerate;
	ret = ioctl(fd, ADC_IOCTL_SET_SAMPLERATE, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return ret;
	}
	printf("Samplerate is set to %d\n", val);

	printf("setting sample length\n");
	val = 131072;
	ret = ioctl(fd, ADC_IOCTL_SET_LENGTH, &val);
	if (ret == -1) {                 
		printf("ioctl error!\n");
		return ret;
	}
	printf("Length is set to %d\n", val);

	printf("setting range\n");
	val = ADS8556_CR_RANGE_2VREF;
	ret = ioctl(fd, ADC_IOCTL_SET_RANGE, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return ret;
	}
	printf("range is set to %d\n", val);

	printf("setting vref\n");
	val = ADS8556_CR_VREF_2_5;
	ret = ioctl(fd, ADC_IOCTL_SET_VREF, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return ret;
	}
	printf("vref is set to %d\n", val);

	printf("setting refadc\n");
	val = (1024) - 1;
	ret = ioctl(fd, ADC_IOCTL_SET_REFDAC, &val);
	if (ret == -1) {
		printf("ioctl error!\n");
		return ret;
	}
	printf("refadc is set to %d\n", val);

	printf("start data acquistion\n");
	ret = fsync(fd);
	if (ret == -1) {
		printf("acquistion error!\n");
		return ret;
	}
	printf("acquistion finished\n");

	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
	}
	ad_range=5;
	return 0;
}

int get_deta_and_fft()
{
	FFT_N=samplerate*(int)the_zero;
	compx* s1 = new compx[FFT_N];
	compx* s2 = new compx[FFT_N];
	compx* s3 = new compx[FFT_N];
	compx* s4 = new compx[FFT_N];
	compx* s5 = new compx[FFT_N];
	compx* s6 = new compx[FFT_N];


	printf("%d\n",FFT_N);

	int i=0;
	int16_t deta[6*FFT_N]={0};
	float magnitude1[FFT_N/2+1]={0};
        float magnitude2[FFT_N/2+1]={0};
        float magnitude3[FFT_N/2+1]={0};
        float magnitude4[FFT_N/2+1]={0};
        float magnitude5[FFT_N/2+1]={0};
        float magnitude6[FFT_N/2+1]={0};
	float angle1[FFT_N/2+1]={0};
        float angle2[FFT_N/2+1]={0};
        float angle3[FFT_N/2+1]={0};
        float angle4[FFT_N/2+1]={0};
        float angle5[FFT_N/2+1]={0};
        float angle6[FFT_N/2+1]={0};
	float power1=0,power2=0,power3=0;
	float phase_sub_avg[3]={0};
	float lvur=0,iur=0,rms_avg=0,max_sub=0;
	int f1=0,f2=0,f3=0,f4=0,f5=0,f6=0;
	float ch1_rms=0,ch2_rms=0,ch3_rms=0,ch4_rms=0,ch5_rms=0,ch6_rms=0;
	float ch1_thd=0,ch2_thd=0,ch3_thd=0,ch4_thd=0,ch5_thd=0,ch6_thd=0;
	float ch1_val=0,ch2_val=0,ch3_val=0,ch4_val=0,ch5_val=0,ch6_val=0;

	int fd;
	int ret;
	
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		return -1;
	}

	ret = fsync(fd);
	if (ret == -1) {
		printf("acquistion error!\n");
		return ret;
	}

	ret = read(fd, deta, 12*FFT_N/(int)the_zero);
	if (ret == -1) {
		printf("read error!\n");
		return ret;
	}

	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
	}
/*
i=0;
while(i < FFT_N/the_zero)
{
	printf("%X %X %X %X %X %X %d\n",deta[i*6],deta[i*6+1],deta[i*6+2],deta[i*6+3],deta[i*6+4],deta[i*6+5],i);
	i=i+1;
}
*/
i=0;

	while(i < FFT_N/the_zero)                                        //給結構體賦值
	{
	
		s1[i].real = deta[i*6];         				      //實部為正弦波FFT_N點采樣，賦值為1
		s1[i].imag =0;                                        //虛部為0
		s2[i].real = deta[i*6+1];
		s2[i].imag =0; 
                s3[i].real = deta[i*6+2];
                s3[i].imag =0;
                s4[i].real = deta[i*6+3];
                s4[i].imag =0;
                s5[i].real = deta[i*6+4];
                s5[i].imag =0;
                s6[i].real = deta[i*6+5];
                s6[i].imag =0;

	i=i+1;
	}
	while(i < FFT_N)                                        //給結構體賦值
	{
	
		s1[i].real = 0;         				      //實部為正弦波FFT_N點采樣，賦值為1
		s1[i].imag =0;                                        //虛部為0
		s2[i].real = 0;
		s2[i].imag =0; 
                s3[i].real = 0;
                s3[i].imag =0;
                s4[i].real = 0;
                s4[i].imag =0;
                s5[i].real = 0;
                s5[i].imag =0;
                s6[i].real = 0;
                s6[i].imag =0;

	i=i+1;
	}
	
/*	i=0;
	for(i;i<FFT_N;i++)
	{
		printf("%d:%f %f %f %f %f %f \n",i,s1[i].real,s2[i].real,s3[i].real,s4[i].real,s5[i].real,s6[i].real);
		printf("%x %x %x %x %x %x \n",s1[i].imag,s2[i].imag,s3[i].imag,s4[i].imag,s5[i].imag,s6[i].imag);

	}
*/

	FFT(s1);                                                //進行快速福利葉變換
        FFT(s2);
        FFT(s3);
        FFT(s4);
        FFT(s5);
        FFT(s6);
  //    printf("%d:%f  %f  %f  %f  %f  %f\n",i,s1[0].real,s2[0].real,s3[0].real,s4[0].real,s5[0].real,s6[0].real);

	float i_sc=ad_range*i_scale;
	float v_sc=ad_range*v_scale;


	for(i = 0; i <= (FFT_N/2); i++)                             //求變換後結果的模值，存入復數的實部部分
	{
		magnitude1[i] = sqrt(s1[i].real*s1[i].real + s1[i].imag * s1[i].imag)*v_sc/32768/FFT_N;
                magnitude2[i] = sqrt(s2[i].real*s2[i].real + s2[i].imag * s2[i].imag)*v_sc/32768/FFT_N;
                magnitude3[i] = sqrt(s3[i].real*s3[i].real + s3[i].imag * s3[i].imag)*v_sc/32768/FFT_N;
                magnitude4[i] = sqrt(s4[i].real*s4[i].real + s4[i].imag * s4[i].imag)*i_sc/32768/FFT_N;
                magnitude5[i] = sqrt(s5[i].real*s5[i].real + s5[i].imag * s5[i].imag)*i_sc/32768/FFT_N;
                magnitude6[i] = sqrt(s6[i].real*s6[i].real + s6[i].imag * s6[i].imag)*i_sc/32768/FFT_N;
	
	angle1[i]=atan2(s1[i].real,s1[i].imag);
        angle2[i]=atan2(s2[i].real,s2[i].imag);
        angle3[i]=atan2(s3[i].real,s3[i].imag);
        angle4[i]=atan2(s4[i].real,s4[i].imag);
        angle5[i]=atan2(s5[i].real,s5[i].imag);
        angle6[i]=atan2(s6[i].real,s6[i].imag);
//	printf("%d:%f  %f  %f  %f  %f  %f ",i,s1[i].real,s2[i].real,s3[i].real,s4[i].real,s5[i].real,s6[i].real);
//	printf("  %f  %f  %f  %f  %f  %f\n",s1[i].imag,s2[i].imag,s3[i].imag,s4[i].imag,s5[i].imag,s6[i].imag);
//	printf("%d:%f  %f  %f  %f  %f  %f\n ",i,angle1[i],angle2[i],angle3[i],angle4[i],angle5[i],angle6[i]);
	
	}
	for(i = 1; i <= (FFT_N/2) ; i++)
	{
		ch1_rms=(ch1_rms+pow((magnitude1[i]*2),2));
                ch2_rms=(ch2_rms+pow((magnitude2[i]*2),2));
                ch3_rms=(ch3_rms+pow((magnitude3[i]*2),2));
                ch4_rms=(ch4_rms+pow((magnitude4[i]*2),2));
                ch5_rms=(ch5_rms+pow((magnitude5[i]*2),2));
                ch6_rms=(ch6_rms+pow((magnitude6[i]*2),2));
	}

	ch1_rms=sqrt(pow(magnitude1[0],2)+(ch1_rms/2))*sqrt(the_zero);
        ch2_rms=sqrt(pow(magnitude2[0],2)+(ch2_rms/2))*sqrt(the_zero);        
        ch3_rms=sqrt(pow(magnitude3[0],2)+(ch3_rms/2))*sqrt(the_zero);
        ch4_rms=sqrt(pow(magnitude4[0],2)+(ch4_rms/2))*sqrt(the_zero);
        ch5_rms=sqrt(pow(magnitude5[0],2)+(ch5_rms/2))*sqrt(the_zero);
        ch6_rms=sqrt(pow(magnitude6[0],2)+(ch6_rms/2))*sqrt(the_zero);

	printf("ch1_rms=%f\n",ch1_rms);
        printf("ch2_rms=%f\n",ch2_rms);
        printf("ch3_rms=%f\n",ch3_rms);
        printf("ch4_rms=%f\n",ch4_rms);
        printf("ch5_rms=%f\n",ch5_rms);
        printf("ch6_rms=%f\n",ch6_rms);

	report_buf[0]=ch1_rms;
	report_buf[1]=ch2_rms;
	report_buf[2]=ch3_rms;
	report_buf[3]=ch4_rms;
	report_buf[4]=ch5_rms;
	report_buf[5]=ch6_rms;			

	f1=0;f2=0;f3=0;f4=0;f5=0;f6=0;
	for(i = 0;i <= (FFT_N/2) ; i++)
	{
		if(magnitude1[i]>ch1_val)
		{
			f1=i;
			ch1_val=magnitude1[i];
		}
                if(magnitude2[i]>ch2_val)
                {
                        f2=i;
                        ch2_val=magnitude2[i];
		}
                if(magnitude3[i]>ch3_val)
                {
                        f3=i;
                        ch3_val=magnitude3[i];
                }

                if(magnitude4[i]>ch4_val)
                {
                        f4=i;
                        ch4_val=magnitude4[i];
                }
                if(magnitude5[i]>ch5_val)
                {
                        f5=i;
                        ch5_val=magnitude5[i];
                }
                if(magnitude6[i]>ch6_val)
                {
                        f6=i;
                        ch6_val=magnitude6[i];
                }

	}


	printf("ch1:%fHz\nch2:%fHz\nch3:%fHz\nch4:%fHz\nch5:%fHz\nch6:%fHz\n",f1/the_zero,f2/the_zero,f3/the_zero,f4/the_zero,f5/the_zero,f6/the_zero);

	report_buf[6]=f1/the_zero;
	report_buf[7]=f2/the_zero;
	report_buf[8]=f3/the_zero;
	report_buf[9]=f4/the_zero;
	report_buf[10]=f5/the_zero;
	report_buf[11]=f6/the_zero;

/*	
	printf("ch1_angle:%f(in%dHz)\n",angle1[f1],f1);
        printf("ch2_angle:%f(in%dHz)\n",angle2[f2],f2);
        printf("ch3_angle:%f(in%dHz)\n",angle3[f3],f3);
        printf("ch4_angle:%f(in%dHz)\n",angle4[f4],f4);
        printf("ch5_angle:%f(in%dHz)\n",angle5[f5],f5);
        printf("ch6_angle:%f(in%dHz)\n",angle6[f6],f6);
*/
	if(f1>0)
	{
		i=f1*2;
		ch1_val=0;
		while(i<(FFT_N/2))
		{
			ch1_val=ch1_val+pow(magnitude1[i],2);
			i=i+f1;	
		}
		ch1_thd=100*sqrt(ch1_val)/magnitude1[f1];
		printf("ch1_thd=%f%%\n",ch1_thd);
		report_buf[12]=ch1_thd;
	}
	else
	{
		printf("ch1_thd=NaN\n");
		report_buf[12]=-1;
	}

        if(f2>0)
        {
                i=f2*2;
                ch2_val=0;
                while(i<(FFT_N/2))
                {
                        ch2_val=ch2_val+pow(magnitude2[i],2);
                        i=i+f2;
                }
                ch2_thd=100*sqrt(ch2_val)/magnitude2[f2];
                printf("ch2_thd=%f%%\n",ch2_thd);
		report_buf[13]=ch2_thd;
        }
        else
        {
                printf("ch2_thd=NaN\n");
		report_buf[13]=-1;
        }

        if(f3>0)
        {
                i=f3*2;
                ch3_val=0;
                while(i<(FFT_N/2))
                {
                        ch3_val=ch3_val+pow(magnitude3[i],2);
                        i=i+f3;
                }
                ch3_thd=100*sqrt(ch3_val)/magnitude3[f3];
                printf("ch3_thd=%f%%\n",ch3_thd);
		report_buf[14]=ch3_thd;
        }
        else
        {
                printf("ch3_thd=NaN\n");
		report_buf[14]=-1;
        }

        if(f4>0)
        {
                i=f4*2;
                ch4_val=0;
                while(i<(FFT_N/2))
                {
                        ch4_val=ch4_val+pow(magnitude4[i],2);
                        i=i+f4;
                }
                ch4_thd=100*sqrt(ch4_val)/magnitude4[f4];
                printf("ch4_thd=%f%%\n",ch4_thd);
		report_buf[15]=ch4_thd;
        }
        else
        {
                printf("ch4_thd=NaN\n");
		report_buf[15]=-1;
        }

        if(f5>0)
        {
                i=f5*2;
                ch5_val=0;
                while(i<(FFT_N/2))
                {
                        ch5_val=ch5_val+pow(magnitude5[i],2);
                        i=i+f5;
                }
                ch5_thd=100*sqrt(ch5_val)/magnitude5[f5];
                printf("ch5_thd=%f%%\n",ch5_thd);
		report_buf[16]=ch5_thd;
        }
        else
        {
                printf("ch5_thd=NaN\n");
		report_buf[16]=-1;
        }

        if(f6>0)
        {
                i=f6*2;
                ch6_val=0;
                while(i<(FFT_N/2))
                {
                        ch6_val=ch6_val+pow(magnitude6[i],2);
                        i=i+f6;
                }
                ch6_thd=100*sqrt(ch6_val)/magnitude6[f6];
                printf("ch6_thd=%f%%\n",ch6_thd);
		report_buf[17]=ch6_thd;
        }
        else
        {
                printf("ch6_thd=NaN\n");
		report_buf[17]=-1;
        }
	
	if(f1>0)
	{
		if(f1==f4)
		{
			power1=2*magnitude1[f1]*magnitude4[f1]*cos(angle1[f1]-angle4[f1])*pow(the_zero,2);

			printf("power1:%f(in%dHz)\n",power1,f1);
			report_buf[18]=power1;		
		}
		else
		{
			printf("power1:NaN\n");
			report_buf[18]=-1;
		}
	}

	if(f1==0)
	{
                if(f1==f4)
                {
                        power1=magnitude1[f1]*magnitude4[f1]*pow(the_zero,2);

                        printf("power1:%f(in%dHz)\n",power1,f1);
			report_buf[18]=power1;
                }
                else
                {
                        printf("power1:NaN\n");
			report_buf[18]=-1;
                }
	}

        if(f2>0)
        {
                if(f2==f5)
                {
                        power2=2*magnitude2[f2]*magnitude5[f2]*cos(angle2[f2]-angle5[f2])*pow(the_zero,2);
	
                        printf("power2:%f(in%dHz)\n",power2,f2);
			report_buf[19]=power2;
                }
                else
                {
                        printf("power2:NaN\n");
			report_buf[19]=-1;
                }
        }

        if(f2==0)
        {
                if(f2==f5)
                {
                        power2=magnitude2[f2]*magnitude5[f2]*pow(the_zero,2);
			
                        printf("power2:%f(in%dHz)\n",power2,f2);
			report_buf[19]=power2;
                }
                else
                {
                        printf("power2:NaN\n");
			report_buf[19]=-1;
                }
        }

        if(f3>0)
        {
                if(f3==f6)
                {
                        power3=2*magnitude3[f3]*magnitude6[f3]*cos(angle3[f3]-angle6[f3])*pow(the_zero,2);
			
                        printf("power3:%f(in%dHz)\n",power3,f3);
			report_buf[20]=power3;
                }
                else
                {
                        printf("power3:NaN\n");
			report_buf[20]=-1;
                }
        }

        if(f3==0)
        {
                if(f3==f6)
                {
                        power3=magnitude3[f3]*magnitude6[f3]*pow(the_zero,2);
			
                        printf("power3:%f(in%dHz)\n",power3,f3);
			report_buf[20]=power3;
                }
                else
                {
                        printf("power3:NaN\n");
			report_buf[20]=-1;
                }
        }

	rms_avg=(ch1_rms+ch2_rms+ch3_rms)/3;
	phase_sub_avg[0]=fabs(ch1_rms-rms_avg);
        phase_sub_avg[1]=fabs(ch2_rms-rms_avg);
        phase_sub_avg[2]=fabs(ch3_rms-rms_avg);	
	for(i=0;i<3;i++)
	{
//		printf("%f\n",phase_sub_avg[i]);
		if(phase_sub_avg[i]>max_sub)
		{
			max_sub=phase_sub_avg[i];
		}
	}
	lvur=100*max_sub/rms_avg;
	printf("LVUR:%f%%\n",lvur);
	report_buf[21]=lvur;

	max_sub=0;
        rms_avg=(ch4_rms+ch5_rms+ch6_rms)/3;
        phase_sub_avg[0]=fabs(ch4_rms-rms_avg);
        phase_sub_avg[1]=fabs(ch5_rms-rms_avg);
        phase_sub_avg[2]=fabs(ch6_rms-rms_avg);
        for(i=0;i<3;i++)
        {
                if(phase_sub_avg[i]>max_sub)
                {
                        max_sub=phase_sub_avg[i];
                }
        }
        iur=100*max_sub/rms_avg;
        printf("IUR:%f%%\n",iur);
	report_buf[22]=iur;

/*
	memset(s1,0,sizeof(s1));
	memset(s2,0,sizeof(s2));
	memset(s3,0,sizeof(s3));
	memset(s4,0,sizeof(s4));
	memset(s5,0,sizeof(s5));
	memset(s6,0,sizeof(s6));
*/


	delete[] s1;
	delete[] s2;
	delete[] s3;
	delete[] s4;
	delete[] s5;
	delete[] s6;


	memset(deta,0,sizeof(deta));
	memset(magnitude1,0,sizeof(magnitude1));
        memset(magnitude2,0,sizeof(magnitude2));
        memset(magnitude3,0,sizeof(magnitude3));
        memset(magnitude4,0,sizeof(magnitude4));
        memset(magnitude5,0,sizeof(magnitude5));
        memset(magnitude6,0,sizeof(magnitude6));
	memset(angle1,0,sizeof(angle1));
        memset(angle2,0,sizeof(angle2));
        memset(angle3,0,sizeof(angle3));
        memset(angle4,0,sizeof(angle4));
        memset(angle5,0,sizeof(angle5));
        memset(angle6,0,sizeof(angle6));
	power1=0,power2=0,power3=0;
	memset(phase_sub_avg,0,sizeof(phase_sub_avg));
	lvur=0,iur=0,rms_avg=0,max_sub=0;
	f1=0,f2=0,f3=0,f4=0,f5=0,f6=0;
	ch1_rms=0,ch2_rms=0,ch3_rms=0,ch4_rms=0,ch5_rms=0,ch6_rms=0;
	ch1_thd=0,ch2_thd=0,ch3_thd=0,ch4_thd=0,ch5_thd=0,ch6_thd=0;
	ch1_val=0,ch2_val=0,ch3_val=0,ch4_val=0,ch5_val=0,ch6_val=0;



	return 0;
}


//----------------------------------------------------------------------------





void printCookies(const Http::Request& req) {
    auto cookies = req.cookies();
    std::cout << "Cookies: [" << std::endl;
    const std::string indent(4, ' ');
    for (const auto& c: cookies) {
        std::cout << indent << c.name << " = " << c.value << std::endl;
    }
    std::cout << "]" << std::endl;
}

namespace Generic {

void handleReady(const Rest::Request&, Http::ResponseWriter response) {
    response.send(Http::Code::Ok, "3");
}
//----------------------------------------------------------------------
void _get_deta_and_fft(const Rest::Request&, Http::ResponseWriter response) {
	printf("in _get_deta_and_fft function\n");
	int re;
	re=setting_samplength(samplerate);
	if (re < 0) {
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("setting_samplelength function error!");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	re=get_deta_and_fft();
	Json::Value FFT_report;
	
	Json::Value rms;
	rms["ch1_rms"] = Json::Value(report_buf[0]); 
	rms["ch2_rms"] = Json::Value(report_buf[1]);
	rms["ch3_rms"] = Json::Value(report_buf[2]); 
	rms["ch4_rms"] = Json::Value(report_buf[3]);
	rms["ch5_rms"] = Json::Value(report_buf[4]); 
	rms["ch6_rms"] = Json::Value(report_buf[5]);

	Json::Value freq;
	freq["ch1_freq"] = Json::Value(report_buf[6]); 
	freq["ch2_freq"] = Json::Value(report_buf[7]);
	freq["ch3_freq"] = Json::Value(report_buf[8]); 
	freq["ch4_freq"] = Json::Value(report_buf[9]);
	freq["ch5_freq"] = Json::Value(report_buf[10]); 
	freq["ch6_freq"] = Json::Value(report_buf[11]);

	Json::Value thd;
	thd["ch1_thd"] = Json::Value(report_buf[12]); 
	thd["ch2_thd"] = Json::Value(report_buf[13]);
	thd["ch3_thd"] = Json::Value(report_buf[14]); 
	thd["ch4_thd"] = Json::Value(report_buf[15]);
	thd["ch5_thd"] = Json::Value(report_buf[16]); 
	thd["ch6_thd"] = Json::Value(report_buf[17]);

	Json::Value power;
	power["power1"] = Json::Value(report_buf[18]); 
	power["power2"] = Json::Value(report_buf[19]);
	power["power3"] = Json::Value(report_buf[20]);
 
	Json::Value unblance_rate;
	unblance_rate["LVUR"] = Json::Value(report_buf[21]);
	unblance_rate["IUR"] = Json::Value(report_buf[22]); 

	

	
	FFT_report["rms"]=Json::Value(rms);
	FFT_report["frequency"]=Json::Value(freq);
	FFT_report["thd"]=Json::Value(thd);
	FFT_report["power"]=Json::Value(power);
	FFT_report["unblance_rate"]=Json::Value(unblance_rate);
	FFT_report["spectrum_resolution"] = Json::Value(1.0/the_zero);
	FFT_report["input_range"] = Json::Value(ad_range);
	FFT_report["v_scale"] = Json::Value(v_scale);
	FFT_report["i_scale"] = Json::Value(i_scale);
	Json::StyledWriter sw;
				
        response.send(Http::Code::Ok, sw.write(FFT_report), MIME(Application, Json));
	memset(report_buf,0,sizeof(report_buf));
	printf("out _get_deta_and_fft function\n");
}
void sampling_rate(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in sampling_rate function\n");
	auto rate = req.splatAt(0).as<std::string>();
	int re,er=0;
	int old_samplerate=samplerate;
		if(rate=="1k"){
			samplerate=1*1024;
			re=setting_samplerate();
			}
		else if(rate=="2k"){
			samplerate=2*1024;
			re=setting_samplerate();
			}
		else if(rate=="4k"){
			samplerate=4*1024;
			re=setting_samplerate();
			}
		else if(rate=="8k"){
			samplerate=8*1024;
			re=setting_samplerate();
			}
		else if(rate=="16k"){
			samplerate=16*1024;
			re=setting_samplerate();
			}
		else if(rate=="32k"){
			samplerate=32*1024;
			re=setting_samplerate();
			}
		else if(rate=="64k"){
			samplerate=64*1024;
			re=setting_samplerate();
			}
		else if(rate=="128k"){
			samplerate=128*1024;
			re=setting_samplerate();
			}	
		else{
			er=-1;
			}
	
	if(er==-1){

		Json::Value err;
		err["setting_sampling_rate"] = Json::Value("error");
		err["options"].append("/1k");
		err["options"].append("/2k");
		err["options"].append("/4k");
		err["options"].append("/8k");
		err["options"].append("/16k");
		err["options"].append("/32k");
		err["options"].append("/64k");
		err["options"].append("/128k");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	if(re==-1){
		samplerate=old_samplerate;
		Json::Value err;
		err["setting_sampling_rate"] = Json::Value("fail");
		err["sampling_rate"] = Json::Value(samplerate);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else{
		Json::Value rep;
		rep["setting_sampling_rate"] = Json::Value("success");
		rep["sampling_rate"] = Json::Value(samplerate);	
		if((samplerate*the_zero)>=262144){
			the_zero=131072/samplerate;
		}
		rep["the_zero"] = Json::Value(the_zero);	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));
	}
	printf("out sampling_rate function\n");
}

void sampling_length(const Rest::Request& req, Http::ResponseWriter response) {


	printf("in sampling_length function\n");
	auto length = req.splatAt(0).as<int>();
	int re;
	
	re=setting_samplength(length);
	if (re == 0) {
		Json::Value rep;
		rep["setting_sampling_length"] = Json::Value("success");
		rep["sampling_length"] = Json::Value(length);
		samplelength=length;
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));
	}
	else if (re == -1) {
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("open device error!");
		err["sampling_length"] = Json::Value(samplelength);	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	else if (re == -2) {                 
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("ioctl error!");
		err["sampling_length"] = Json::Value(samplelength);	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	
	else if (re == -3) {
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("close error!");
		err["sampling_length"] = Json::Value(samplelength);	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else {
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("function error!");
		err["sampling_length"] = Json::Value(samplelength);	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	printf("out sampling_length function\n");
	
}

void get_deta(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in get_deta function\n");
	int fd;
	int ret;
	int16_t deta[6*samplelength]={0};
	int re;
	re=setting_samplength(samplelength);

	if (re < 0) {
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("setting_samplelength function error!");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		Json::Value err;
		err["get_deta"] = Json::Value("open device error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	ret = fsync(fd);
	if (ret == -1) {
		printf("acquistion error!\n");
		Json::Value err;
		err["get_deta"] = Json::Value("acquistion error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	ret = read(fd, deta, 12*samplelength);
	if (ret == -1) {
		printf("read error!\n");
		Json::Value err;
		err["get_deta"] = Json::Value("read error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
		Json::Value err;
		err["get_deta"] = Json::Value("close error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	Json::Value rep;
	int ii=0;
	while(ii<samplelength){

		rep["ch1"].append(deta[ii*6]);
		//printf("%d",deta[ii*6]);
		rep["ch2"].append(deta[ii*6+1]);
		rep["ch3"].append(deta[ii*6+2]);
		rep["ch4"].append(deta[ii*6+3]);
		rep["ch5"].append(deta[ii*6+4]);
		rep["ch6"].append(deta[ii*6+5]);
		ii=ii+1;
	}
	rep["input_range"] = Json::Value(ad_range);
	rep["v_scale"] = Json::Value(v_scale);
	rep["i_scale"] = Json::Value(i_scale);
	rep["sample_rate"] = Json::Value(samplerate);
	rep["sample_length"] = Json::Value(samplelength);
	Json::FastWriter fw;
	response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));
	printf("out get_deta function\n");
}

void setting_the_zero(const Rest::Request& req, Http::ResponseWriter response) {
	
	printf("in setting_the_zero function\n");
	auto zeros = req.splatAt(0).as<int>();
	int re;
	if(zeros!=1){
		re=zeros%2;
	}
	else if(zeros==1){
		re=0;
	}
	if((re!=0)||((samplerate*zeros)>=262144)){
		printf("setting_the_zero error!\n");
		Json::Value err;
		err["set_the_zero"] = Json::Value("error!");
		err["suggestion"].append("the_zero must be multiple of 2 or equal to 1");
		err["suggestion"].append("the_zero x samplerate must lass than 262144");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else {
		the_zero=zeros;
		Json::Value rep;
		rep["setting_the_zero"] = Json::Value("success");
		rep["the_zero"] = Json::Value(the_zero);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));
		
	}
	printf("out setting_the_zero function\n");
}

void setting_ad_input_range(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in set_ad_input_range function\n");
	auto r = req.splatAt(0).as<int>();
	int re,rr;
	
	switch(r){
		case 5:
			vref=ADS8556_CR_VREF_2_5;
			range=ADS8556_CR_RANGE_2VREF;
			re=set_ad_input_range();
			rr=5;
			break;
		case 6:
			vref=ADS8556_CR_VREF_3;
			range=ADS8556_CR_RANGE_2VREF;
			re=set_ad_input_range();
			rr=6;
			break;
		case 10:
			vref=ADS8556_CR_VREF_2_5;
			range=ADS8556_CR_RANGE_4VREF;
			re=set_ad_input_range();
			rr=10;		
			break;
		case 12:
			vref=ADS8556_CR_VREF_3;
			range=ADS8556_CR_RANGE_4VREF;
			re=set_ad_input_range();
			rr=12;
			break;
		default:
			re=9487;
			break;
	}
	if(re==0){
		ad_range=rr;
		Json::Value rep;
		rep["set_ad_input_range"] = Json::Value("success");
		rep["ad_range"] = Json::Value(ad_range);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));
	}
	else if(re==-1){
		Json::Value err;
		err["set_ad_input_range"] = Json::Value("open device error!");
		err["ad_range"] = Json::Value(ad_range);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else if(re==-2){
		Json::Value err;
		err["set_ad_input_range"] = Json::Value("ADC_IOCTL_SET_RANGE error!");
		err["ad_range"] = Json::Value(ad_range);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else if(re==-3){
		Json::Value err;
		err["set_ad_input_range"] = Json::Value("ADC_IOCTL_SET_VREF error!");
		err["ad_range"] = Json::Value(ad_range);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else if(re==-4){
		Json::Value err;
		err["set_ad_input_range"] = Json::Value("close device error!");
		err["ad_range"] = Json::Value(ad_range);
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else if(re==9487){
		Json::Value err;
		err["set_ad_input_range"] = Json::Value("cerror!");
		err["ad_range"] = Json::Value(ad_range);
		err["options"].append("5");
		err["options"].append("6");
		err["options"].append("10");
		err["options"].append("12");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	else{
		Json::Value err;
		err["set_ad_input_range"] = Json::Value("function cerror!");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	printf("out setting_the_zero function\n");

}

void setting_voltage_scale(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in setting_voltage_scale\n");
	auto r = req.splatAt(0).as<float>();
	v_scale=r;
	Json::Value rep;
	rep["set_voltage_scale"] = Json::Value("success");
	rep["v_scale"] = Json::Value(v_scale);
	Json::StyledWriter sw;
	response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));

	printf("out setting_voltage_scale function\n");
}


void setting_current_scale(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in setting_current_scale\n");
	auto r = req.splatAt(0).as<float>();
	i_scale=r;
	Json::Value rep;
	rep["setting_current_scale"] = Json::Value("success");
	rep["i_scale"] = Json::Value(i_scale);
	Json::StyledWriter sw;
	response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));

	printf("out setting_current_scale function\n");
}

void _sync(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in sync function\n");
	memset(sync_deta,0,sizeof(sync_deta));
	int fd;
	int ret;
	int re;
	re=setting_samplength(samplelength);

	if (re < 0) {
		Json::Value err;
		err["setting_sampling_length"] = Json::Value("setting_samplelength function error!");
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	fd = open("/dev/adc_ti_ads8556", O_RDONLY);
	if (fd == -1) {
		printf("open device error!\n");
		Json::Value err;
		err["sync"] = Json::Value("open device error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	ret = fsync(fd);
	if (ret == -1) {
		printf("acquistion error!\n");
		Json::Value err;
		err["sync"] = Json::Value("acquistion error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	ret = read(fd, sync_deta, 12*samplelength);
	if (ret == -1) {
		printf("read error!\n");
		Json::Value err;
		err["sync"] = Json::Value("read error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}

	ret = close(fd);
	if (ret != 0) {
		printf("close error!\n");
		Json::Value err;
		err["sync"] = Json::Value("close error!");	
		Json::StyledWriter sw;
		response.send(Http::Code::Ok, sw.write(err), MIME(Application, Json));
	}
	sync_rate=samplerate;
	sync_length=samplelength;
	Json::Value rep;
	rep["sync"] = Json::Value("success");
	rep["sample_rate"] = Json::Value(sync_rate);
	rep["sample_length"] = Json::Value(sync_length);
	rep["i_scale"] = Json::Value(i_scale);
	rep["v_scale"] = Json::Value(v_scale);
	Json::StyledWriter sw;
	response.send(Http::Code::Ok, sw.write(rep), MIME(Application, Json));
	printf("out sync function\n");
}

void _get(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in get function\n");
	if((sync_rate>0)&&(sync_length>0)){
		Json::Value rep;
		int ii=0;
		while(ii<sync_length){

			rep["ch1"].append(sync_deta[ii*6]);
			//printf("%d",sync_deta[ii*6]);
			rep["ch2"].append(sync_deta[ii*6+1]);
			rep["ch3"].append(sync_deta[ii*6+2]);
			rep["ch4"].append(sync_deta[ii*6+3]);
			rep["ch5"].append(sync_deta[ii*6+4]);
			rep["ch6"].append(sync_deta[ii*6+5]);
			ii=ii+1;
		}
		rep["input_range"] = Json::Value(ad_range);
		rep["v_scale"] = Json::Value(v_scale);
		rep["i_scale"] = Json::Value(i_scale);
		rep["sample_rate"] = Json::Value(samplerate);
		rep["sample_length"] = Json::Value(samplelength);
		Json::FastWriter fw;
		response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));
	}
	else{
		Json::Value err;
		err["error"] = Json::Value("no sync deta!");
		Json::FastWriter fw;
		response.send(Http::Code::Ok, fw.write(err), MIME(Application, Json));
	}
	sync_rate=0;
	sync_length=0;
	memset(sync_deta,0,sizeof(sync_deta));
	printf("out get function\n");
}

void get_rms(const Rest::Request& req, Http::ResponseWriter response) {
	printf("in get_rms function\n");
	auto r = req.splatAt(0).as<int>();
	char cc = req.splatAt(0).as<char>();
	int i=0;
	if((sync_rate>0)&&(sync_length>0)){
		if(r==0){

			float ch1_rms,ch2_rms,ch3_rms,ch4_rms,ch5_rms,ch6_rms=0;

			for(i = 0; i < sync_length; i++){

				ch1_rms=ch1_rms+pow((sync_deta[i*6]/32768),2);
				ch2_rms=ch2_rms+pow((sync_deta[i*6+1]/32768),2);
				ch3_rms=ch3_rms+pow((sync_deta[i*6+2]/32768),2);
				ch4_rms=ch4_rms+pow((sync_deta[i*6+3]/32768),2);
				ch5_rms=ch5_rms+pow((sync_deta[i*6+4]/32768),2);
				ch6_rms=ch6_rms+pow((sync_deta[i*6+5]/32768),2);
			}
			ch1_rms=sqrt(ch1_rms/sync_length)*ad_range*v_scale;
			ch2_rms=sqrt(ch2_rms/sync_length)*ad_range*v_scale;
			ch3_rms=sqrt(ch3_rms/sync_length)*ad_range*v_scale;
			ch4_rms=sqrt(ch4_rms/sync_length)*ad_range*i_scale;
			ch5_rms=sqrt(ch5_rms/sync_length)*ad_range*i_scale;
			ch6_rms=sqrt(ch6_rms/sync_length)*ad_range*i_scale;
			Json::Value rep;
			rep["ch1_rms"] = Json::Value(ch1_rms); 
			rep["ch2_rms"] = Json::Value(ch2_rms);
			rep["ch3_rms"] = Json::Value(ch3_rms); 
			rep["ch4_rms"] = Json::Value(ch4_rms);
			rep["ch5_rms"] = Json::Value(ch5_rms); 
			rep["ch6_rms"] = Json::Value(ch6_rms);
			Json::FastWriter fw;
			response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));
		}
		else if((r>=1)&&(r<=6)){

			float ch_rms=0;
			i=0;
			int j=r-1;
			for(i = 0; i < sync_length; i++){

				ch_rms=ch_rms+pow((sync_deta[i*6+j]/32768),2);
			}
			if((r>=1)&&(r<=3)){
				ch_rms=sqrt(ch_rms/sync_length)*ad_range*v_scale;
			}
			else{
				ch_rms=sqrt(ch_rms/sync_length)*ad_range*i_scale;

			}
			Json::Value rep;
			
			rep["rms"] = Json::Value(ch_rms); 
	
			Json::FastWriter fw;
			response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));

	
		}
		else{
		Json::Value err;
		err["error"] = Json::Value("parameter error!");
		Json::FastWriter fw;
		response.send(Http::Code::Ok, fw.write(err), MIME(Application, Json));
	}


	}
	else{
		Json::Value err;
		err["error"] = Json::Value("no sync deta!");
		Json::FastWriter fw;
		response.send(Http::Code::Ok, fw.write(err), MIME(Application, Json));
	}

	printf("out get_rms function\n");
}

void get_phase(const Rest::Request& req, Http::ResponseWriter response) {
/*	printf("in get_rms function\n");
	auto r = req.splatAt(0).as<int>();
	auto cc = req.splatAt(0).as<char>();
	int i=0;
	if((sync_rate>0)&&(sync_length>0)){
		if(r==0){

			float ch1_rms,ch2_rms,ch3_rms,ch4_rms,ch5_rms,ch6_rms=0;

			for(i = 0; i < sync_length; i++){

				ch1_rms=ch1_rms+pow((sync_deta[i*6]/32768),2);
				ch2_rms=ch2_rms+pow((sync_deta[i*6+1]/32768),2);
				ch3_rms=ch3_rms+pow((sync_deta[i*6+2]/32768),2);
				ch4_rms=ch4_rms+pow((sync_deta[i*6+3]/32768),2);
				ch5_rms=ch5_rms+pow((sync_deta[i*6+4]/32768),2);
				ch6_rms=ch6_rms+pow((sync_deta[i*6+5]/32768),2);
			}
			ch1_rms=sqrt(ch1_rms)*v_sc;
			ch2_rms=sqrt(ch2_rms)*v_sc;
			ch3_rms=sqrt(ch3_rms)*v_sc;
			ch4_rms=sqrt(ch4_rms)*i_sc;
			ch5_rms=sqrt(ch5_rms)*i_sc;
			ch6_rms=sqrt(ch6_rms)*i_sc;
			Json::Value rep;
			rep["ch1_rms"] = Json::Value(ch1_rms); 
			rep["ch2_rms"] = Json::Value(ch2_rms);
			rep["ch3_rms"] = Json::Value(ch3_rms); 
			rep["ch4_rms"] = Json::Value(ch4_rms);
			rep["ch5_rms"] = Json::Value(ch5_rms); 
			rep["ch6_rms"] = Json::Value(ch6_rms);
			Json::FastWriter fw;
			response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));
		}
		else if((r>=1)&&(r<=6)){

			float ch_rms=0;
			i=0;
			j=r-1;
			for(i = 0; i < sync_length; i++){

				ch_rms=ch_rms+pow((sync_deta[i*6+j]/32768),2);
			}
			if((r>=1)&&(r<=3)){
				ch_rms=sqrt(ch_rms)*v_sc;
			}
			else{
				ch_rms=sqrt(ch_rms)*i_sc;

			}
			Json::Value rep;
			rep["ch"+cc+"_rms"] = Json::Value(ch_rms); 
	
			Json::FastWriter fw;
			response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));

	
		}

	}
	else{
		Json::Value err;
		err["error"] = Json::Value("no sync deta!");
		Json::FastWriter fw;
		response.send(Http::Code::Ok, fw.write(err), MIME(Application, Json));
	}

	printf("out get_rms function\n");
*/
}

void help(const Rest::Request& req, Http::ResponseWriter response) {
	
	printf("in help function\n");
	Json::Value rep;
	rep["Functions"].append("/get_deta_and_fft");
	rep["Functions"].append("/sampling_rate/*");
	rep["Functions"].append("/sampling_length/*");
	rep["Functions"].append("/get_deta");
	rep["Functions"].append("/set_the_zero/*");
	rep["Functions"].append("/set_ad_input_range/*");
	rep["Functions"].append("/info");
	rep["Functions"].append("/set_voltage_scale/*");
	rep["Functions"].append("/set_current_scale/*");
	rep["Functions"].append("/sync");
	rep["Functions"].append("/get");
	rep["Functions"].append("/get_rms/*");
	rep["Functions"].append("/get_phase/*");
	Json::FastWriter fw;
	response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));
	printf("out help function\n");
}

void info(const Rest::Request& req, Http::ResponseWriter response) {
	
	printf("in info function\n");
	Json::Value rep;
	rep["sampling_rate"] = Json::Value(samplerate);
	rep["sampling_length"] = Json::Value(samplelength);
	rep["the_zero"] = Json::Value(the_zero);
	rep["ad_input_range"] = Json::Value(ad_range);
	rep["v_scale"] = Json::Value(v_scale);
	rep["i_scale"] = Json::Value(i_scale);
	if((sync_length==0)&&(sync_rate==0)){

		rep["sync_data"] = Json::Value(0);
	
	}
	else {

		rep["sync_data"] = Json::Value(1);
	}
	Json::FastWriter fw;
	response.send(Http::Code::Ok, fw.write(rep), MIME(Application, Json));
	printf("out info function\n");
}

//----------------------------------------------------------------------
}

class StatsEndpoint {
public:
    StatsEndpoint(Address addr)
        : httpEndpoint(std::make_shared<Http::Endpoint>(addr))
    { }

    void init(size_t thr = 2) {
        auto opts = Http::Endpoint::options()
            .threads(thr);
        httpEndpoint->init(opts);
        setupRoutes();
    }

    void start() {
        httpEndpoint->setHandler(router.handler());
        httpEndpoint->serve();
    }

private:
    void setupRoutes() {
        using namespace Rest;

        Routes::Post(router, "/record/:name/:value?", Routes::bind(&StatsEndpoint::doRecordMetric, this));
        Routes::Get(router, "/value/:name", Routes::bind(&StatsEndpoint::doGetMetric, this));
        Routes::Get(router, "/ready", Routes::bind(&Generic::handleReady));
        Routes::Get(router, "/auth", Routes::bind(&StatsEndpoint::doAuth, this));

	Routes::Get(router, "/get_deta_and_fft", Routes::bind(&Generic::_get_deta_and_fft));
	Routes::Get(router, "/sampling_rate/*", Routes::bind(&Generic::sampling_rate));
	Routes::Get(router, "/sampling_length/*", Routes::bind(&Generic::sampling_length));
	Routes::Get(router, "/get_deta", Routes::bind(&Generic::get_deta));
	Routes::Get(router, "/set_the_zero/*", Routes::bind(&Generic::setting_the_zero));
	Routes::Get(router, "/set_ad_input_range/*", Routes::bind(&Generic::setting_ad_input_range));
	Routes::Get(router, "/set_voltage_scale/*", Routes::bind(&Generic::setting_voltage_scale));
	Routes::Get(router, "/set_current_scale/*", Routes::bind(&Generic::setting_current_scale));
	Routes::Get(router, "/sync", Routes::bind(&Generic::_sync));
	Routes::Get(router, "/get", Routes::bind(&Generic::_get));
	Routes::Get(router, "/get_rms/*", Routes::bind(&Generic::get_rms));
	Routes::Get(router, "/get_phase/*", Routes::bind(&Generic::get_phase));
	Routes::Get(router, "/help", Routes::bind(&Generic::help));
	Routes::Get(router, "/info", Routes::bind(&Generic::info));
    }

    void doRecordMetric(const Rest::Request& request, Http::ResponseWriter response) {
        auto name = request.param(":name").as<std::string>();

        Guard guard(metricsLock);
        auto it = std::find_if(metrics.begin(), metrics.end(), [&](const Metric& metric) {
            return metric.name() == name;
        });

        int val = 1;
        if (request.hasParam(":value")) {
            auto value = request.param(":value");
            val = value.as<int>();
        }

        if (it == std::end(metrics)) {
            metrics.push_back(Metric(std::move(name), val));
            response.send(Http::Code::Created, std::to_string(val));
        }
        else {
            auto &metric = *it;
            metric.incr(val);
            response.send(Http::Code::Ok, std::to_string(metric.value()));
        }

    }

    void doGetMetric(const Rest::Request& request, Http::ResponseWriter response) {
        auto name = request.param(":name").as<std::string>();

        Guard guard(metricsLock);
        auto it = std::find_if(metrics.begin(), metrics.end(), [&](const Metric& metric) {
            return metric.name() == name;
        });

        if (it == std::end(metrics)) {
            response.send(Http::Code::Not_Found, "Metric does not exist");
        } else {
            const auto& metric = *it;
            response.send(Http::Code::Ok, std::to_string(metric.value()));
        }

    }

    void doAuth(const Rest::Request& request, Http::ResponseWriter response) {
        printCookies(request);
        response.cookies()
            .add(Http::Cookie("lang", "en-US"));
        response.send(Http::Code::Ok);
    }

    class Metric {
    public:
        Metric(std::string name, int initialValue = 1)
            : name_(std::move(name))
            , value_(initialValue)
        { }

        int incr(int n = 1) {
            int old = value_;
            value_ += n;
            return old;
        }

        int value() const {
            return value_;
        }

        std::string name() const {
            return name_;
        }
    private:
        std::string name_;
        int value_;
    };

    typedef std::mutex Lock;
    typedef std::lock_guard<Lock> Guard;
    Lock metricsLock;
    std::vector<Metric> metrics;

    std::shared_ptr<Http::Endpoint> httpEndpoint;
    Rest::Router router;
};

int main(int argc, char *argv[]) {
    
    initial_ads8556();

    Port port(9080);

    int thr = 2;

    if (argc >= 2) {
        port = std::stol(argv[1]);

        if (argc == 3)
            thr = std::stol(argv[2]);
    }

    Address addr("192.168.80.10", port);

    cout << "Cores = " << hardware_concurrency() << endl;
    cout << "Using " << thr << " threads" << endl;

    StatsEndpoint stats(addr);

    stats.init(thr);
    stats.start();
}
