/**    
  * @author  Liu heng
  * һ�׿������˲�������RoboMaster��̳  
  *   һά�������˲���                     
  *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲��� 
  *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
  *          ʹ��ʾ��                                             
  *          extKalman p;                  //����һ���������˲����ṹ��                                                 
  *          float SersorData;             //��Ҫ�����˲�������                                          
  *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����                                                  
  *          while(1)                                                                
  *          {                                                                            
  *             SersorData = sersor();                     //��ȡ����                                           
  *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�                                                                            
  *          }                                                                            
  */

#include "kalman.h"
#include "rp_config.h" 
/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
		p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;							  //���Ԥ����x(k|k)
}

void kalman_pid_update(kalman_pid_struct_t *kal_pid,extKalman_t *date)
{


}
kalman_pid_t kal_pid[KAL_CNT]=
{
	[kal_standI] = {
		.Stand_speed_kal.Q = 1,
		.Stand_speed_kal.R = 40,
	},
	[kal_locat_pout] = {
		.Locat_speed_kal.Q = 1,
		.Locat_speed_kal.R = 400,
	},

};
 void kalman_pid_init(kalman_pid_struct_t * self)
{
	kalman_pid_t *KAL = self->info;
//	if(self->id == kal_standI)
//	{
		KalmanCreate(&KAL->Stand_speed_kal,KAL->Stand_speed_kal.Q,KAL->Stand_speed_kal.R);
//	}
//	else if(self->id == kal_locat_pout)
//	{
//		KalmanCreate(&KAL->Locat_speed_kal,KAL->Locat_speed_kal.Q,KAL->Locat_speed_kal.R);
//	}
}



kalman_pid_struct_t KAL_Pid[KAL_CNT] =
{
	[kal_standI] ={
		.info = &kal_pid[kal_standI],
		.id = kal_standI,
		.init=kalman_pid_init,
	},
	[kal_locat_pout] ={
		.info = &kal_pid[kal_locat_pout],
		.id = kal_locat_pout,
		.init = kalman_pid_init,
	}

};
