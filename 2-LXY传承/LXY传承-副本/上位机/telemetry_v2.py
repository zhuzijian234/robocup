"""V2 wire codec, bounded CONFIG assembly, handshake and diagnostic report.
No serial reader is hidden in this module: callers own their port.
"""
import struct
import math
import json
import time
import secrets
from collections import Counter

LAYOUT = "rc26-mask31-v2.1"
PARAMETERS = "kp kp2 kp3 kd kd2 kd3 sp_kp sp_ki spd disr disl yr yl ystra ysel angle".split()
KEYS = {1:"layout",2:"build",3:"lidar",4:"algorithm",5:"input_kind",6:"baud",
        7:"rate",8:"timeout_ms",9:"health_ms",10:"units",11:"dma_bytes",
        12:"point_capacity",13:"geometry",14:"lidar_config"}
KEYS.update({100+i:n for i,n in enumerate(PARAMETERS)})
HEALTH = "uptime_ms input_age_ms valid_age_ms input_seq control_seq uart_errors dma_errors rx_overflow scan_overflow input_drop tx_drop process_max_us rx_peak param_revision status_flags unsupported_mask".split()
EXTRA = "version session tx_seq control_seq input_end_ms control_end_ms control_dt_us valid_mask updated_mask clipped_mask action_reason motor_state lidar_id algorithm_id param_revision raw_count radar_dps segment host_t".split()


def crc16(data):
    crc = 0xffff
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def packet(kind, payload, seq=0, session=1):
    body = struct.pack("<BBHII", 2, kind, 16+len(payload), seq, session)+payload
    return b"\xaa\x55"+body+struct.pack("<H", crc16(body))


def tlv(key, value):
    if isinstance(value, str): typ, data = 4, value.encode()
    elif isinstance(value, float): typ, data = 3, struct.pack("<f", value)
    else: typ, data = 1, struct.pack("<I", value)
    return struct.pack("<HBH", key, typ, len(data))+data


def decode_tlv(data):
    out, pos = {}, 0
    while pos < len(data):
        if len(data)-pos < 5: raise ValueError("truncated TLV")
        key, typ, n = struct.unpack_from("<HBH", data, pos); pos += 5
        if pos+n > len(data) or key in out: raise ValueError("invalid/duplicate TLV")
        raw=data[pos:pos+n]; pos+=n
        if typ in (1,2,3):
            if n!=4: raise ValueError("numeric TLV size")
            value=struct.unpack({1:"<I",2:"<i",3:"<f"}[typ],raw)[0]
            if typ==3 and not math.isfinite(value): raise ValueError("non-finite config")
        elif typ==4: value=raw.decode("utf-8")
        elif typ==5: value=raw.hex()
        else: raise ValueError("unknown TLV type")
        out[key]=value
    return out


class Decoder:
    def __init__(self):
        self.messages=[]; self.configs={}; self.pending={}
        self.last_tx={}; self.missing=0; self.duplicates=0
        self.clock={}; self.segment=0; self.last_identity=None

    def accept(self, frame, names, scales, host_t):
        version, kind, n, seq, session=struct.unpack_from("<BBHII",frame,2)
        if version!=2 or n!=len(frame) or not 16<=n<=256 or crc16(frame[2:-2])!=int.from_bytes(frame[-2:],"little"):
            raise ValueError("V2 frame/CRC")
        if self.last_tx.get(session)==seq:
            self.duplicates+=1;return None
        msg={"type":kind,"tx_seq":seq,"session":session,"host_t":host_t}
        payload=frame[14:-2]
        rec=None
        if kind==1:
            if n!=108: raise ValueError("CONTROL length")
            cs,inp,end,dt,valid,updated,clipped=struct.unpack_from("<7I",frame,66)
            action,motor,lidar,algorithm=frame[94:98]
            rev,raw,speed=struct.unpack_from("<IHH",frame,98)
            if (valid|updated|clipped)&0x80000000 or clipped&0x70000000 or clipped&~valid:
                raise ValueError("mask")
            if action>6 or motor>3: raise ValueError("enum")
            if action!=1 and updated&5:raise ValueError("non-PD err/mode marked updated")
            vals=struct.unpack_from("<26h",frame,14)
            if valid&(1<<2) and not 0<=vals[2]<=9: raise ValueError("PD enum")
            if valid&(1<<20) and vals[20] not in (0,1): raise ValueError("flag enum")
            identity=(session,rev,lidar,algorithm)
            if identity!=self.last_identity: self.segment+=1; self.last_identity=identity
            # Millisecond modulo delta; backward/discontinuous timestamps start a segment.
            state=self.clock.get(session)
            if valid&(1<<29):
                if state:
                    delta=(end-state[0])&0xffffffff
                    if delta>=0x80000000: self.segment+=1; elapsed=0
                    else: elapsed=state[1]+delta
                else: elapsed=0
                self.clock[session]=(end,elapsed)
                t=elapsed/1000
            else: t=math.nan
            rec={name: vals[i]/scales[i] if valid&(1<<i) else math.nan for i,name in enumerate(names)}
            rec.update(version=2,session=session,seq=seq,tx_seq=seq,t=t,control_seq=cs,
                input_end_ms=inp,control_end_ms=end,control_dt_us=dt,valid_mask=valid,
                updated_mask=updated,clipped_mask=clipped,action_reason=action,motor_state=motor,
                lidar_id=lidar,algorithm_id=algorithm,param_revision=rev,raw_count=raw,radar_dps=speed,
                segment=self.segment,host_t=host_t)
            msg.update(rec)
        elif kind==2:
            if n!=80: raise ValueError("HEALTH length")
            msg.update(zip(HEALTH,struct.unpack("<16I",payload)))
            if msg["status_flags"]&~255 or msg["unsupported_mask"]&~0x3fff: raise ValueError("HEALTH flags")
        elif kind==3:
            if len(payload)<12: raise ValueError("CONFIG prefix")
            cid,rev,index,count=struct.unpack_from("<IIHH",payload)
            if not 1<=count<=64 or index>=count: raise ValueError("CONFIG bounds")
            ident=(session,cid,rev)
            if ident not in self.pending:
                if len(self.pending)>=4:self.pending.pop(next(iter(self.pending)))
                self.pending[ident]=(count,{})
            total,chunks=self.pending[ident]
            if count!=total or (index in chunks and chunks[index]!=payload[12:]): raise ValueError("CONFIG conflict")
            chunks[index]=payload[12:]
            if len(chunks)==count:
                cfg=decode_tlv(b"".join(chunks[i] for i in range(count)))
                if cfg.get(1)!=LAYOUT: raise ValueError("unsupported layout")
                if not set(KEYS)<=set(cfg): raise ValueError("incomplete registry")
                self.configs[(session,rev)]={KEYS.get(k,str(k)):v for k,v in cfg.items()}
                del self.pending[ident]
            msg.update(config_id=cid,param_revision=rev,chunk_index=index,chunk_count=count)
        elif kind==4:
            if len(payload)<12: raise ValueError("EVENT prefix")
            ms,rev,code,size=struct.unpack_from("<IIHH",payload)
            if size!=len(payload)-12: raise ValueError("EVENT body")
            body=payload[12:];msg.update(event_ms=ms,param_revision=rev,event_code=code)
            if code==1:
                if len(body)!=12 or body[2:4]!=bytes([3,4]):raise ValueError("parameter event")
                key=struct.unpack_from("<H",body)[0];old,new=struct.unpack_from("<ff",body,4)
                if not all(map(math.isfinite,[old,new])):raise ValueError("parameter finite")
                msg.update(key=key,old=old,new=new)
                prior=self.configs.get((session,(rev-1)&0xffffffff))
                if prior and key in KEYS:
                    cfg=dict(prior);cfg[KEYS[key]]=new;self.configs[(session,rev)]=cfg
            else: msg["body_hex"]=body.hex()
        else: msg["body_hex"]=payload.hex()
        prev=self.last_tx.get(session)
        if prev is not None:
            gap=(seq-prev-1)&0xffffffff
            if seq==prev:self.duplicates+=1;return None
            if gap<0x80000000:self.missing+=gap
            else:self.segment+=1
        self.last_tx[session]=seq
        self.messages.append(msg)
        return rec


class Handshake:
    """Caller feeds parser first, then calls poll; commands are strictly sequential."""
    def __init__(self, send, parser, rate=1, session=None):
        self.send=send;self.parser=parser;self.session=session or secrets.randbelow(0xffffffff)+1
        self.rate=rate;self.stage=0;self.cursor=len(parser.ascii_lines)
        self.deadline=0;self.done=False;self.error=None;self.info=None
        self.commands=["info",f"session {self.session}","tele 3",f"rate {rate}","getcfg"]

    def start(self,now): self._send(now)
    def _send(self,now):
        if self.send(self.commands[self.stage]+"\n") is False:raise ValueError("send failed")
        self.deadline=now+8
    def poll(self,now):
        if self.done:return
        lines=self.parser.ascii_lines[self.cursor:];self.cursor=len(self.parser.ascii_lines)
        for item in lines:
            line=item["text"]
            if line.startswith("ERR "):raise ValueError("设备拒绝协商: "+line)
            if self.stage==0:
                if not line.startswith("INFO "):continue
                if f"layout={LAYOUT}" not in line or "proto=1,2" not in line:raise ValueError("V2 capability/layout mismatch")
                self.info=line
            elif self.stage>=len(self.commands) or line!="OK "+self.commands[self.stage]:continue
            self.stage+=1
            if self.stage<len(self.commands):self._send(now)
            else: self.deadline=now+15
            break
        if self.stage==len(self.commands):
            matches=[cfg for (sid,_),cfg in self.parser.v2.configs.items() if sid==self.session]
            if matches and matches[-1]["rate"]==self.rate:
                self.done=True;return
        if now>self.deadline:raise TimeoutError("V2协商/配置超时，未进入正式采集")


def analyze(parser,field_names):
    preroll=getattr(parser,"meta",{}).get("pre_roll_s",0)
    rows=[r for r in parser.records if r.get("version")==2 and (r.get("host_t") is None or r["host_t"]>=preroll)]
    periods=[r["control_dt_us"]/1000 for r in rows if r["valid_mask"]&(1<<30)]
    errors=[r["Speed_now"]-r["Speed_mubiao"] for r in rows if math.isfinite(r["Speed_now"]) and math.isfinite(r["Speed_mubiao"])]
    jumps=[]
    for a,b in zip(rows,rows[1:]):
        if (a["segment"]==b["segment"] and (b["control_seq"]-a["control_seq"])&0xffffffff==1
            and all(math.isfinite(r["servo_pwm"]) for r in (a,b))):
            jumps.append(abs(b["servo_pwm"]-a["servo_pwm"]))
    pd=[r for r in rows if r["action_reason"]==1 and r["valid_mask"]&5==5 and r["updated_mask"]&5==5]
    near_sat=sum(abs(r["err"])>=490 for r in pd if not r["clipped_mask"]&1)
    clipped_fields={name:sum(bool(r["clipped_mask"]&(1<<i)) for r in rows) for i,name in enumerate(field_names)}
    unknown=sum((r["session"],r["param_revision"]) not in parser.v2.configs for r in rows)
    return dict(v2=True,n_frames=len(rows),bad_frames=parser.bad_frames,dropped=parser.v2.missing,
        duration_s=parser.capture_duration,periods=periods,speed_error=sum(errors)/len(errors) if errors else None,
        actions=dict(Counter(r["action_reason"] for r in rows)),max_pwm_step=max(jumps,default=None),
        pd_modes=dict(Counter(int(r["pid_select"]) for r in pd)),near_saturation=near_sat,
        clipped_fields=clipped_fields,
        unknown_config=unknown,configs=parser.v2.configs,health=[m for m in parser.v2.messages if m["type"]==2],
        rows=rows,received_bytes=parser.received_bytes)


def report(result,path,name=""):
    p=result["periods"];dur=result["duration_s"]
    lines=[f"# V2遥测报告 — {name}","",f"CONTROL（正式区间）：{result['n_frames']}；CRC/结构失败候选：{result['bad_frames']}",
      f"tx_seq缺口估计：{result['dropped']}（全部V2消息；不等于雷达丢包）",
      f"配置未知的CONTROL：{result['unknown_config']}；这些区间不能追溯控制参数。",
      "", "## 控制周期与状态", "",
      f"有效设备周期样本：{len(p)}；范围ms：{min(p) if p else None} ～ {max(p) if p else None}",
      f"动作分布：{result['actions']}（1 PD，2保持，3/4强制左/右，5感知无效）",
      f"有效且本次更新的PD模式分布：{result['pd_modes']}",
      f"PD误差近饱和样本（|err|≥490，排除编码限幅）：{result['near_saturation']}",
      f"编码限幅字段计数：{ {k:v for k,v in result['clipped_fields'].items() if v} }",
      f"按逐帧目标计算的平均速度误差：{result['speed_error']}",
      f"连续同配置控制帧的最大PWM差：{result['max_pwm_step']}"]
    if dur:lines += [f"含协商阶段的主机接收吞吐：{result['received_bytes']/dur:.1f} B/s；采集总时长：{dur:.3f}s"]
    lines += ["", "## HEALTH", "",f"收到{len(result['health'])}条；末条：", "```json",
              json.dumps(result['health'][-1] if result['health'] else {},ensure_ascii=False,indent=2),"```",
              "unsupported_mask指示尚未实现的统计，不能把占位0解释成零错误。",
              "", "## 配置", "", "```json",
              json.dumps({f"{k[0]}:{k[1]}":v for k,v in result['configs'].items()},ensure_ascii=False,indent=2),"```",
              "", "说明：时间戳为设备接收/处理时刻，不是激光发射时刻。跨会话、参数版本、序号缺口不计算连续差分。",
              "rate>1遗漏的控制结果无法重建。invalid/未更新字段不能作为本次PD证据；未做时钟对齐，不估算单程延迟。"]
    with open(path,"w",encoding="utf-8") as f:f.write("\n".join(lines))
