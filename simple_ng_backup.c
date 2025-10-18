/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage: simple_ng IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * This is a minimal test.
 */

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
   ecx_contextt context;
   char *iface;
   uint8 group;
   int roundtrip_time;
   uint8 map[4096];
} Fieldbus;

static void
fieldbus_initialize(Fieldbus *fieldbus, char *iface)
{
   /* Let's start by 0-filling `fieldbus` to avoid surprises */
   memset(fieldbus, 0, sizeof(*fieldbus));

   fieldbus->iface = iface;
   fieldbus->group = 0;
   fieldbus->roundtrip_time = 0;
}

static int
fieldbus_roundtrip(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_timet start, end, diff;
   int wkc;

   context = &fieldbus->context;

   start = osal_current_time();
   ecx_send_processdata(context);
   wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
   end = osal_current_time();
   osal_time_diff(&start, &end, &diff);
   fieldbus->roundtrip_time = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

   return wkc;
}

static boolean
fieldbus_start(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;

   printf("Initializing SOEM on '%s'... ", fieldbus->iface);
   if (!ecx_init(context, fieldbus->iface))
   {
      printf("no socket connection\n");
      return FALSE;
   }
   printf("done\n");

   printf("Finding autoconfig slaves... ");
   if (ecx_config_init(context) <= 0)
   {
      printf("no slaves found\n");
      return FALSE;
   }
   printf("%d slaves found\n", context->slavecount);

   printf("Sequential mapping of I/O... ");
   ecx_config_map_group(context, fieldbus->map, fieldbus->group);
   printf("mapped %dO+%dI bytes from %d segments",
          grp->Obytes, grp->Ibytes, grp->nsegments);
   if (grp->nsegments > 1)
   {
      /* Show how slaves are distributed */
      for (i = 0; i < grp->nsegments; ++i)
      {
         printf("%s%d", i == 0 ? " (" : "+", grp->IOsegment[i]);
      }
      printf(" slaves)");
   }
   printf("\n");

   printf("Configuring distributed clock... ");
   ecx_configdc(context);
   printf("done\n");

   printf("Waiting for all slaves in safe operational... ");
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   printf("done\n");

   printf("Send a roundtrip to make outputs in slaves happy... ");
   fieldbus_roundtrip(fieldbus);
   printf("done\n");

   printf("Setting operational state..");
   /* Act on slave 0 (a virtual slave used for broadcasting) */
   slave = context->slavelist;
   slave->state = EC_STATE_OPERATIONAL;
   ecx_writestate(context, 0);
   /* Poll the result ten times before giving up */
   for (i = 0; i < 10; ++i)
   {
      printf(".");
      fieldbus_roundtrip(fieldbus);
      ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
      if (slave->state == EC_STATE_OPERATIONAL)
      {
         printf(" all slaves are now operational\n");
         return TRUE;
      }
   }

   printf(" failed,");
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->state != EC_STATE_OPERATIONAL)
      {
         printf(" slave %d is 0x%04X (AL-status=0x%04X %s)",
                i, slave->state, slave->ALstatuscode,
                ec_ALstatuscode2string(slave->ALstatuscode));
      }
   }
   printf("\n");

   return FALSE;
}

static void
fieldbus_stop(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_slavet *slave;

   context = &fieldbus->context;
   /* Act on slave 0 (a virtual slave used for broadcasting) */
   slave = context->slavelist;

   printf("Requesting init state on all slaves... ");
   slave->state = EC_STATE_INIT;
   ecx_writestate(context, 0);
   printf("done\n");

   printf("Close socket... ");
   ecx_close(context);
   printf("done\n");
}

static boolean
fieldbus_dump(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   //uint32 n;
   int wkc, expected_wkc;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;

   wkc = fieldbus_roundtrip(fieldbus);
   expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;
   printf("%6d usec  WKC %d", fieldbus->roundtrip_time, wkc);
   if (wkc < expected_wkc)
   {
      printf(" wrong (expected %d)\n", expected_wkc);
      return FALSE;
   }

   /*printf("  O:");
   for (n = 0; n < grp->Obytes; ++n)
   {
      printf(" %02X", grp->outputs[n]);
   }
   printf("  I:");
   for (n = 0; n < grp->Ibytes; ++n)
   {
      printf(" %02X", grp->inputs[n]);
   }*/
  // 1.1 解析DataNo（UINT16，小端字节序：inputs[1]是高字节，inputs[0]是低字节）
   uint16_t DataNo = (grp->inputs[1] << 8) | grp->inputs[0];

   // 1.2 解析Fx（REAL32，小端字节序：inputs[2]低字节，inputs[5]高字节）
   uint32_t fx_raw = (grp->inputs[5] << 24) |  // 第5字节（最高位）
                     (grp->inputs[4] << 16) |  // 第4字节
                     (grp->inputs[3] << 8)  |  // 第3字节
                     grp->inputs[2];           // 第2字节（最低位）
   float Fx;
   memcpy(&Fx, &fx_raw, sizeof(Fx));  // 安全转换：避免指针对齐问题

   // 1.3 解析Fy（REAL32，inputs[6]~inputs[9]）
   uint32_t fy_raw = (grp->inputs[9] << 24) | 
                     (grp->inputs[8] << 16) | 
                     (grp->inputs[7] << 8)  | 
                     grp->inputs[6];
   float Fy;
   memcpy(&Fy, &fy_raw, sizeof(Fy));

   // 1.4 解析Fz（REAL32，inputs[10]~inputs[13]）
   uint32_t fz_raw = (grp->inputs[13] << 24) | 
                     (grp->inputs[12] << 16) | 
                     (grp->inputs[11] << 8)  | 
                     grp->inputs[10];
   float Fz;
   memcpy(&Fz, &fz_raw, sizeof(Fz));

   // 1.5 解析Mx（REAL32，inputs[14]~inputs[17]）
   uint32_t mx_raw = (grp->inputs[17] << 24) | 
                     (grp->inputs[16] << 16) | 
                     (grp->inputs[15] << 8)  | 
                     grp->inputs[14];
   float Mx;
   memcpy(&Mx, &mx_raw, sizeof(Mx));

   // 1.6 解析My（REAL32，inputs[18]~inputs[21]）
   uint32_t my_raw = (grp->inputs[21] << 24) | 
                     (grp->inputs[20] << 16) | 
                     (grp->inputs[19] << 8)  | 
                     grp->inputs[18];
   float My;
   memcpy(&My, &my_raw, sizeof(My));

   // 1.7 解析Mz（REAL32，inputs[22]~inputs[25]）
   uint32_t mz_raw = (grp->inputs[25] << 24) | 
                     (grp->inputs[24] << 16) | 
                     (grp->inputs[23] << 8)  | 
                     grp->inputs[22];
   float Mz;
   memcpy(&Mz, &mz_raw, sizeof(Mz));

   // 1.8 打印解析后的实际数据（换行避免覆盖，保留原时间戳）
   printf("Parsed Data: DataNo=%d | Fx=%.3f N | Fy=%.3f N | Fz=%.3f N | Mx=%.3f Nm | My=%.3f Nm | Mz=%.3f Nm\n",DataNo, Fx-16.35, Fy-6.70, Fz+12.35, Mx, My, Mz);
   // printf("  T: %lld\r", (long long)context->DCtime);
   return TRUE;
}

static void
fieldbus_check_state(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;
   grp->docheckstate = FALSE;
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->group != fieldbus->group)
      {
         /* This slave is part of another group: do nothing */
      }
      else if (slave->state != EC_STATE_OPERATIONAL)
      {
         grp->docheckstate = TRUE;
         if (slave->state == EC_STATE_SAFE_OP + EC_STATE_ERROR)
         {
            printf("* Slave %d is in SAFE_OP+ERROR, attempting ACK\n", i);
            slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ecx_writestate(context, i);
         }
         else if (slave->state == EC_STATE_SAFE_OP)
         {
            printf("* Slave %d is in SAFE_OP, change to OPERATIONAL\n", i);
            slave->state = EC_STATE_OPERATIONAL;
            ecx_writestate(context, i);
         }
         else if (slave->state > EC_STATE_NONE)
         {
            if (ecx_reconfig_slave(context, i, EC_TIMEOUTRET))
            {
               slave->islost = FALSE;
               printf("* Slave %d reconfigured\n", i);
            }
         }
         else if (!slave->islost)
         {
            ecx_statecheck(context, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE)
            {
               slave->islost = TRUE;
               printf("* Slave %d lost\n", i);
            }
         }
      }
      else if (slave->islost)
      {
         if (slave->state != EC_STATE_NONE)
         {
            slave->islost = FALSE;
            printf("* Slave %d found\n", i);
         }
         else if (ecx_recover_slave(context, i, EC_TIMEOUTRET))
         {
            slave->islost = FALSE;
            printf("* Slave %d recovered\n", i);
         }
      }
   }

   if (!grp->docheckstate)
   {
      printf("All slaves resumed OPERATIONAL\n");
   }
}

int main(int argc, char *argv[])
{
   Fieldbus fieldbus;

   if (argc != 2)
   {
      ec_adaptert *adapter = NULL;
      ec_adaptert *head = NULL;
      printf("Usage: simple_ng IFNAME1\n"
             "IFNAME1 is the NIC interface name, e.g. 'eth0'\n");

      printf("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
      return 1;
   }

   fieldbus_initialize(&fieldbus, argv[1]);
   if (fieldbus_start(&fieldbus))
   {
      int i, min_time, max_time;
      min_time = max_time = 0;
      for (i = 1; i <= 10000; ++i)
      {
         printf("Iteration %4d:", i);
         if (!fieldbus_dump(&fieldbus))
         {
            fieldbus_check_state(&fieldbus);
         }
         else if (i == 1)
         {
            min_time = max_time = fieldbus.roundtrip_time;
         }
         else if (fieldbus.roundtrip_time < min_time)
         {
            min_time = fieldbus.roundtrip_time;
         }
         else if (fieldbus.roundtrip_time > max_time)
         {
            max_time = fieldbus.roundtrip_time;
         }
         osal_usleep(8000);  // 这里修改频率
      }
      printf("\nRoundtrip time (usec): min %d max %d\n", min_time, max_time);
      fieldbus_stop(&fieldbus);
   }

   return 0;
}
