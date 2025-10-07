/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         ec_config_map(&IOmap);

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
                /* cyclic loop */
            for(i = 1; i <= 10000; i++)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {

                        needlf = TRUE;
                        if (iloop >= 26)
                        {
                           // 1.1 解析DataNo（UINT16，小端字节序：inputs[1]是高字节，inputs[0]是低字节）
                           uint16_t DataNo = (ec_slave[0].inputs[1] << 8) | ec_slave[0].inputs[0];

                           // 1.2 解析Fx（REAL32，小端字节序：inputs[2]低字节，inputs[5]高字节）
                           uint32_t fx_raw = (ec_slave[0].inputs[5] << 24) |  // 第5字节（最高位）
                                             (ec_slave[0].inputs[4] << 16) |  // 第4字节
                                             (ec_slave[0].inputs[3] << 8)  |  // 第3字节
                                             ec_slave[0].inputs[2];           // 第2字节（最低位）
                           float Fx;
                           memcpy(&Fx, &fx_raw, sizeof(Fx));  // 安全转换：避免指针对齐问题

                           // 1.3 解析Fy（REAL32，inputs[6]~inputs[9]）
                           uint32_t fy_raw = (ec_slave[0].inputs[9] << 24) | 
                                             (ec_slave[0].inputs[8] << 16) | 
                                             (ec_slave[0].inputs[7] << 8)  | 
                                             ec_slave[0].inputs[6];
                           float Fy;
                           memcpy(&Fy, &fy_raw, sizeof(Fy));

                           // 1.4 解析Fz（REAL32，inputs[10]~inputs[13]）
                           uint32_t fz_raw = (ec_slave[0].inputs[13] << 24) | 
                                             (ec_slave[0].inputs[12] << 16) | 
                                             (ec_slave[0].inputs[11] << 8)  | 
                                             ec_slave[0].inputs[10];
                           float Fz;
                           memcpy(&Fz, &fz_raw, sizeof(Fz));

                           // 1.5 解析Mx（REAL32，inputs[14]~inputs[17]）
                           uint32_t mx_raw = (ec_slave[0].inputs[17] << 24) | 
                                             (ec_slave[0].inputs[16] << 16) | 
                                             (ec_slave[0].inputs[15] << 8)  | 
                                             ec_slave[0].inputs[14];
                           float Mx;
                           memcpy(&Mx, &mx_raw, sizeof(Mx));

                           // 1.6 解析My（REAL32，inputs[18]~inputs[21]）
                           uint32_t my_raw = (ec_slave[0].inputs[21] << 24) | 
                                             (ec_slave[0].inputs[20] << 16) | 
                                             (ec_slave[0].inputs[19] << 8)  | 
                                             ec_slave[0].inputs[18];
                           float My;
                           memcpy(&My, &my_raw, sizeof(My));

                           // 1.7 解析Mz（REAL32，inputs[22]~inputs[25]）
                           uint32_t mz_raw = (ec_slave[0].inputs[25] << 24) | 
                                             (ec_slave[0].inputs[24] << 16) | 
                                             (ec_slave[0].inputs[23] << 8)  | 
                                             ec_slave[0].inputs[22];
                           float Mz;
                           memcpy(&Mz, &mz_raw, sizeof(Mz));

                           // 1.8 打印解析后的实际数据（换行避免覆盖，保留原时间戳）
                           printf("\n Parsed Data: DataNo=%d | Fx=%.3f N | Fy=%.3f N | Fz=%.3f N | Mx=%.3f Nm | My=%.3f Nm | Mz=%.3f Nm"PRId64,DataNo, Fx, Fy, Fz, Mx, My, Mz);
                        }
                    }
                    osal_usleep(5000);

                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
