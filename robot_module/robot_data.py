

class JointDataBuffer:
    def __init__(self, num_joints):
        """初始化关节数据缓存"""
        self.num_joints = num_joints
        self.buffer = [[] for _ in range(num_joints)]  # 为每个关节创建独立的缓存列表
        self.max_size = 100  # 设置最大缓存大小
    
    def add_batch_data(self, positions, velocities):
        """批量添加所有关节数据到缓存
        参数:
            positions: 所有关节的位置列表
            velocities: 所有关节的速度列表
        """
        if len(positions) != self.num_joints or len(velocities) != self.num_joints:
            raise ValueError(f"数据长度不匹配，预期 {self.num_joints} 个关节，得到位置 {len(positions)} 个，速度 {len(velocities)} 个")
            
        for joint_id in range(self.num_joints):
            self.buffer[joint_id].append((positions[joint_id], velocities[joint_id]))
            # 如果超过最大缓存大小，移除最早的数据
            if len(self.buffer[joint_id]) > self.max_size:
                self.buffer[joint_id].pop(0)

    def add_data(self, joint_id, position, velocity):
        """添加关节数据到缓存"""
        if 0 <= joint_id < self.num_joints:
            self.buffer[joint_id].append((position, velocity))
            # 如果超过最大缓存大小，移除最早的数据
            if len(self.buffer[joint_id]) > self.max_size:
                self.buffer[joint_id].pop(0)
        else:
            raise ValueError(f"关节ID {joint_id} 超出范围 (0-{self.num_joints-1})")

    def get_latest_data(self, joint_id):
        """获取指定关节的最新数据"""
        if 0 <= joint_id < self.num_joints:
            if self.buffer[joint_id]:
                return self.buffer[joint_id][-1]
            return None
        raise ValueError(f"关节ID {joint_id} 超出范围 (0-{self.num_joints-1})")

    def clear_buffer(self, joint_id=None):
        """清空缓存"""
        if joint_id is None:
            for i in range(self.num_joints):
                self.buffer[i].clear()
        elif 0 <= joint_id < self.num_joints:
            self.buffer[joint_id].clear()
        else:
            raise ValueError(f"关节ID {joint_id} 超出范围 (0-{self.num_joints-1})")

    def pop_earliest_data(self, joint_id):
        """获取并删除指定关节缓存中最早的数据
        参数:
            joint_id: 关节ID
        返回:
            元组 (position, velocity) 或 None（如果缓存为空）
        """
        if 0 <= joint_id < self.num_joints:
            if self.buffer[joint_id]:
                return self.buffer[joint_id].pop(0)
            return None
        raise ValueError(f"关节ID {joint_id} 超出范围 (0-{self.num_joints-1})")