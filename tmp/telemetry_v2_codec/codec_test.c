#include <stdint.h>
#include <float.h>
static uint32_t tx_seq;
static const uint16_t scales[26]={10,1,1,1000,1,1000,1,1000,1000,1,1,1,1,1,1,1,1,1,10,1,1,1,1,1,1,1};
static void put16(uint8_t*p,uint16_t v){p[0]=v;p[1]=v>>8;}
static void put32(uint8_t*p,uint32_t v){put16(p,(uint16_t)v);put16(p+2,(uint16_t)(v>>16));}
uint16_t Diag_CRC(const uint8_t*p,uint16_t n){uint16_t c=0xffff;uint8_t i;while(n--){c^=(uint16_t)*p++<<8;for(i=0;i<8;i++)c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}return c;}
int16_t Diag_Encode(uint8_t i,float v,uint8_t*valid,uint8_t*clipped){
    float limit=0, scaled; double rounded;
    *valid=0;*clipped=0;
    if(i>=26 || !(v<=FLT_MAX && v>=-FLT_MAX))return 0;
    if(i==2 && (v<0 || v>9 || v!=(int)v))return 0;
    if(i==20 && v!=0 && v!=1)return 0;
    if(i==0)limit=600;
    if(i==3 || i==5 || i==7 || i==8)limit=30;
    if(i==4 || i==6)limit=30000;
    if(!limit && i!=18 && v<0)return 0;
    *valid=1;
    if(limit){if(v>limit){v=limit;*clipped=1;}if(v< -limit){v= -limit;*clipped=1;}}
    scaled=v*scales[i];
    if(scaled>32767){*clipped=1;return 32767;}
    if(scaled< -32768){*clipped=1;return -32768;}
    rounded=(double)scaled+(scaled>=0?0.5:-0.5);
    return (int16_t)(int32_t)rounded;
}
void Diag_Finalize(uint8_t*p,uint16_t n){if(n>=16 && p[0]==0xaa && p[1]==0x55 && p[2]==2){put32(p+6,tx_seq++);put16(p+n-2,Diag_CRC(p+2,n-4));}}
static void motor_poll(void){
    uint32_t tail=motor_tail,head=motor_head,seq,rev;uint8_t n=0;uint16_t off;MotorSample *s;
    if(Diag_mode!=3 || !Diag_session || tail==head || !BLE_NormalSpace())return;
    if(head-tail<8 && (uint32_t)(Diag_TimeUs()-motor_ring[tail&63].us)<80000u)return;
    seq=motor_ring[tail&63].seq;rev=motor_ring[tail&63].rev;
    while(n<8 && tail+n!=head && motor_ring[(tail+n)&63].rev==rev && motor_ring[(tail+n)&63].seq==seq+n)n++;
    header(6,(uint16_t)(32+28*n));put16(frame+14,1);put16(frame+16,n);put32(frame+18,seq);put32(frame+22,rev);put32(frame+26,motor_dropped);
    for(n=0,off=30;tail+n!=head && n<8;n++,off+=28){
        s=&motor_ring[(tail+n)&63];if(s->rev!=rev || s->seq!=seq+n)break;
        put32(frame+off,s->us);put16(frame+off+4,s->raw);put16(frame+off+6,s->pwm);
        memcpy(frame+off+8,&s->speed,4);memcpy(frame+off+12,&s->target,4);
        memcpy(frame+off+16,&s->integral,4);memcpy(frame+off+20,&s->prelimit,4);put32(frame+off+24,s->flags);
    }
    if(BLE_Queue(frame,(uint16_t)(32+28*n),0)){__DMB();motor_tail=tail+n;}
}
static void detail_submit(uint32_t elapsed,uint8_t action,uint8_t ok){
    uint8_t i;
    (void)action;
    Diag_detail_u[0]=1;Diag_detail_u[1]=control_seq;Diag_detail_u[2]=Diag_revision;
    Diag_detail_u[5]=elapsed;Diag_detail_u[11]=hold_count;
    (void)ok;
    Diag_detail_u[19]=LEIDA_sync_failures;Diag_detail_u[20]=LEIDA_missing_packets;
    Diag_detail_u[21]=Diag_input_drop;Diag_detail_u[22]=Diag_tx_drop;Diag_detail_u[23]=motor_dropped;
    header(5,176);for(i=0;i<24;i++)put32(frame+14+4*i,Diag_detail_u[i]);
    for(i=0;i<16;i++)memcpy(frame+110+4*i,&Diag_detail_f[i],4);
    BLE_Queue(frame,176,0);
}

volatile int codec_test_result=-1;
static int run(void){
    union{uint32_t bits;float v;}u;uint8_t valid,clipped;int16_t value;unsigned i;
    uint8_t packet[108]={170,85,2,1,108,0,0,0,0,0,1,0,0,0,136,19,165,5,7,0,244,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,100,0,0,0,110,0,0,0,0,0,0,0,255,255,255,63,255,255,255,63,0,0,0,0,1,2,1,1,0,0,0,0,188,1,112,8,122,81};
    const uint8_t expected[108]={170,85,2,1,108,0,0,0,0,0,1,0,0,0,136,19,165,5,7,0,244,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,100,0,0,0,110,0,0,0,0,0,0,0,255,255,255,63,255,255,255,63,0,0,0,0,1,2,1,1,0,0,0,0,188,1,112,8,122,81};
u.bits=0x43fa0000; value=Diag_Encode(0,u.v,&valid,&clipped); if(value!=5000 || valid!=1 || clipped!=0)return 1;
u.bits=0x44228000; value=Diag_Encode(0,u.v,&valid,&clipped); if(value!=6000 || valid!=1 || clipped!=1)return 2;
u.bits=0xc4228000; value=Diag_Encode(0,u.v,&valid,&clipped); if(value!=-6000 || valid!=1 || clipped!=1)return 3;
u.bits=0x3d0d4fdf; value=Diag_Encode(3,u.v,&valid,&clipped); if(value!=35 || valid!=1 || clipped!=0)return 4;
u.bits=0xbd0d4fdf; value=Diag_Encode(3,u.v,&valid,&clipped); if(value!=-35 || valid!=1 || clipped!=0)return 5;
u.bits=0x42200000; value=Diag_Encode(3,u.v,&valid,&clipped); if(value!=30000 || valid!=1 || clipped!=1)return 6;
u.bits=0xbf000000; value=Diag_Encode(3,u.v,&valid,&clipped); if(value!=-500 || valid!=1 || clipped!=0)return 7;
u.bits=0x46fffe00; value=Diag_Encode(1,u.v,&valid,&clipped); if(value!=32767 || valid!=1 || clipped!=0)return 8;
u.bits=0xc54ccccd; value=Diag_Encode(18,u.v,&valid,&clipped); if(value!=-32768 || valid!=1 || clipped!=0)return 9;
u.bits=0x7fc00000; value=Diag_Encode(0,u.v,&valid,&clipped); if(value!=0 || valid!=0 || clipped!=0)return 10;
u.bits=0x7f800000; value=Diag_Encode(0,u.v,&valid,&clipped); if(value!=0 || valid!=0 || clipped!=0)return 11;
u.bits=0xff800000; value=Diag_Encode(0,u.v,&valid,&clipped); if(value!=0 || valid!=0 || clipped!=0)return 12;
u.bits=0x41200000; value=Diag_Encode(2,u.v,&valid,&clipped); if(value!=0 || valid!=0 || clipped!=0)return 13;
    if(Diag_CRC((const uint8_t*)"123456789",9)!=0x29b1)return 50;
    packet[106]=packet[107]=0;Diag_Finalize(packet,108);
    for(i=0;i<108;i++)if(packet[i]!=expected[i])return 100+i;
    return 0;
}
int main(void){codec_test_result=run();return codec_test_result;}
