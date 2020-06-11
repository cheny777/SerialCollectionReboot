#ifndef CIPROTAL_H
#define CIPROTAL_H

extern "C" __declspec(dllexport) bool OnLinkMachine (unsigned char channel, unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4);
extern "C" __declspec(dllexport) bool OnRegCiValue  (unsigned char channel, long * addr,long num,double scan_ts,double send_ts);
extern "C" __declspec(dllexport) bool OnSetCiValue  (unsigned char channel, long * addr,double * values,long num);
extern "C" __declspec(dllexport) long OnGetCiValue  (unsigned char channel, double * values,unsigned long * serial,long max_num);
extern "C" __declspec(dllexport) void OnRelease(unsigned char channel);
extern "C" __declspec(dllexport) void OnReleaseAll();


extern "C" __declspec(dllexport) bool OnLinkMachineAT (unsigned char channel, unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4);
extern "C" __declspec(dllexport) bool OnRegCiValueAT  (unsigned char channel, long * addr,long num,double scan_ts,double send_ts);
extern "C" __declspec(dllexport) bool OnSetCiValueAT  (unsigned char channel, long * addr,double * values,long num);
extern "C" __declspec(dllexport) long OnGetCiValueAT  (unsigned char channel, double * values,unsigned long * serial,long max_num);
extern "C" __declspec(dllexport) void OnReleaseAT(unsigned char channel);
extern "C" __declspec(dllexport) void OnReleaseAllAT();


#endif // CIPROTAL_H
