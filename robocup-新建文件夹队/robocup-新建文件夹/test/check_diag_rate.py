"""执行生产rate命令：保留同值不清转向历史的修正，移除已回退的急弯事件测试。"""
from check_m10p import P, OUT, run

source = (P/'HARDWARE/hc-05/ble_diag.c').read_text(encoding='utf-8')
number = source[source.index('static uint8_t number('):source.index('uint8_t Diag_Command(')]
rate = source[source.index('    if (!strncmp(line, "rate ", 5))'):source.index('static void m10p_health')]
code = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint8_t Diag_rate=1, sample=3, config_pending;
static uint32_t Diag_revision=100;
static unsigned rejected;
static void Diag_Reject(const char *s){(void)s;++rejected;}
static void Send_Bluetooth_Data(const char *s){(void)s;}
#define CHECK(x) do{if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);return 1;}}while(0)
''' + number + '''
static uint8_t rate_command(char *line){uint32_t n;char ack[64];
''' + rate + r'''
int main(void){
 CHECK(rate_command("rate 1") && Diag_revision==100 && sample==3);
 CHECK(rate_command("rate 2") && Diag_revision==101 && Diag_rate==2 && sample==0);
 CHECK(rate_command("rate 0") && rejected==1 && Diag_revision==101);
 config_pending=1;
 CHECK(rate_command("rate 3") && rejected==2 && Diag_rate==2);
 puts("PASS same-value rate preserves revision/history; changed, invalid and busy cases");
 return 0;
}
'''
path = OUT/'diag_rate.c'
path.write_text(code, encoding='utf-8')
run('diag_rate', path)
