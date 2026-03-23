use axstd::vec::Vec;
// 定义函数指针类型别名，方便阅读
// 这代表一个接受 u8 切片且没有返回值的函数
type SerialWriteFn = fn(&[u8]);

pub struct Sts3215 {
    // 这里直接存储函数指针，而不是泛型
    puts: SerialWriteFn,
}

impl Sts3215 {
    /// 初始化：传入具体的发送函数
    pub fn new(puts_func: SerialWriteFn) -> Self {
        Self { puts: puts_func }
    }

    /// 校验和算法: (~sum(data)) & 0xFF
    /// 注意：data 应该只包含 ID, Length, Instruction, Params
    fn checksum(&self, data: &[u8]) -> u8 {
        let mut sum: u32 = 0;
        for &b in data {
            sum += b as u32;
        }
        (!sum) as u8
    }

    /// 发送指令底层实现
    pub fn send_cmd(&self, servo_id: u8, instruction: u8, params: &[u8]) {
        let length = (params.len() + 2) as u8;
        
        // 预分配内存，避免多次重分配
        // 大小 = 2(Header) + 1(ID) + 1(Len) + 1(Instr) + N(Params) + 1(Check)
        let mut pkt = Vec::with_capacity(6 + params.len());

        // 1. Header
        pkt.push(0xFF);
        pkt.push(0xFF);

        // 2. Body (ID, Length, Instruction)
        pkt.push(servo_id);
        pkt.push(length);
        pkt.push(instruction);

        // 3. Parameters
        pkt.extend_from_slice(params);

        // 4. Calculate Checksum
        // Python 代码是: self.checksum(pkt[2:])，即跳过前两个 0xFF
        // 此时 pkt 包含 Header + Body + Params
        let check_sum = self.checksum(&pkt[2..]);
        pkt.push(check_sum);

        // 5. 调用外部传入的函数指针发送
        (self.puts)(&pkt);
    }

    /// 写寄存器 (0x03)
    pub fn write_reg(&self, servo_id: u8, addr: u8, data: &[u8]) {
        let mut params = Vec::with_capacity(1 + data.len());
        params.push(addr);
        params.extend_from_slice(data);
        
        self.send_cmd(servo_id, 0x03, &params);
    }

    /// 移动到绝对位置 (0-4095)
    pub fn move_to_position(&self, servo_id: u8, pos: i32) {
        // Python: max(0, min(4095, int(pos)))
        let pos = pos.clamp(0, 4095) as u16;
        let data = pos.to_le_bytes(); // 转换为小端序
        
        // 0x2A 是目标位置寄存器
        self.write_reg(servo_id, 0x2A, &data);
    }

    /// 角度控制 (0-360度)
    pub fn move_angle(&self, servo_id: u8, angle: u32) {
        let pos = angle * 4095 / 360;
        self.move_to_position(servo_id, pos as i32);
    }

    /// 设置速度
    pub fn set_speed(&self, servo_id: u8, speed: i32) {
        let speed_val = speed as u16;
        let data = speed_val.to_le_bytes();
        
        // 0x2E 是速度寄存器
        self.write_reg(servo_id, 0x2E, &data);
    }
}
