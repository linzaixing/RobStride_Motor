import yaml
from collections import defaultdict


def parse_ros_yaml(yaml_content):
    """解析包含多个ROS轨迹消息的YAML文件"""
    try:
        documents = list(yaml.safe_load_all(yaml_content))
    except yaml.YAMLError as e:
        print(f"YAML解析错误: {e}")
        return None

    if not documents:
        print("警告: 未找到有效的YAML文档")
        return None

    results = {
        'headers': [],
        'goal_ids': [],
        'trajectories': defaultdict(list)
    }

    for doc in documents:
        if not doc or not isinstance(doc, dict):
            print("跳过无效文档: 不是字典类型")
            continue

        try:
            # 解析Header (带默认值)
            header = {
                'seq': doc.get('header', {}).get('seq', 0),
                'stamp': {
                    'secs': doc.get('header', {}).get('stamp', {}).get('secs', 0),
                    'nsecs': doc.get('header', {}).get('stamp', {}).get('nsecs', 0)
                },
                'frame_id': doc.get('header', {}).get('frame_id', '')
            }
            results['headers'].append(header)

            # 解析GoalID (带默认值)
            goal_id = {
                'id': doc.get('goal_id', {}).get('id', 'unknown'),
                'stamp': {
                    'secs': doc.get('goal_id', {}).get('stamp', {}).get('secs', 0),
                    'nsecs': doc.get('goal_id', {}).get('stamp', {}).get('nsecs', 0)
                }
            }
            results['goal_ids'].append(goal_id)

            # 检查是否存在轨迹数据
            if 'goal' not in doc or 'trajectory' not in doc['goal']:
                print(f"警告: 文档 {goal_id['id']} 缺少轨迹数据")
                continue

            traj = doc['goal']['trajectory']
            joint_names = traj.get('joint_names', [])

            trajectory_data = {
                'header': {
                    'seq': traj.get('header', {}).get('seq', 0),
                    'stamp': traj.get('header', {}).get('stamp', {'secs': 0, 'nsecs': 0}),
                    'frame_id': traj.get('header', {}).get('frame_id', '')
                },
                'joint_names': joint_names,
                'points': []
            }

            for point in traj.get('points', []):
                # 为每个关节创建默认值列表
                num_joints = len(joint_names)
                point_data = {
                    'positions': dict(zip(joint_names, point.get('positions', [0] * num_joints))),
                    'velocities': dict(zip(joint_names, point.get('velocities', [0] * num_joints))),
                    'accelerations': dict(zip(joint_names, point.get('accelerations', [0] * num_joints))),
                    'time_from_start': point.get('time_from_start', {'secs': 0, 'nsecs': 0})
                }
                trajectory_data['points'].append(point_data)

            results['trajectories'][goal_id['id']].append(trajectory_data)

        except Exception as e:
            print(f"解析文档时出错: {e}")
            continue

    return results

def paraData():
    # 使用示例
    with open('goal.yaml', 'r') as f:
        yaml_content = f.read()

    parsed_data = parse_ros_yaml(yaml_content)

    # 打印摘要信息
    print(f"解析到 {len(parsed_data['headers'])} 条消息")
    print(f"第一条消息的GoalID: {parsed_data['goal_ids'][0]['id']}")
    print(f"包含的关节: {parsed_data['trajectories'][list(parsed_data['trajectories'].keys())[0]][0]['joint_names']}")

    # # 获取第一条轨迹的第一个点的详细数据
    # first_traj = parsed_data['trajectories'][list(parsed_data['trajectories'].keys())[0]][0]
    # print(f"所有关节{first_traj}")
    # first_point = first_traj['points'][0]
    # print("\n第一个轨迹点的数据示例:")
    # print(f"位置: {first_point['positions']}")
    # print(f"速度: {first_point['velocities']}")
    # print(f"加速度: {first_point['accelerations']}")
    # print(f"时间: {first_point['time_from_start']['secs']}.{first_point['time_from_start']['nsecs']}秒")

    # 获取第i条轨迹的第j个点的详细数据
    print(f"一共{len(parsed_data['trajectories'].keys())}个轨迹")
    for i in range(0, len(parsed_data['trajectories'].keys())):
        first_traj = parsed_data['trajectories'][list(parsed_data['trajectories'].keys())[i]][0]
        print(f"--------------------第{i+1}个轨迹--------------------")
        for j in range(0, len(first_traj['points'])):
            first_point = first_traj['points'][j]
            print(f"\n第{j+1}个轨迹点的数据示例:")
            print(f"位置: {first_point['positions']}")
            print(f"速度: {first_point['velocities']}")
            print(f"加速度: {first_point['accelerations']}")
            print(f"时间: {first_point['time_from_start']['secs']}.{first_point['time_from_start']['nsecs']}秒")

if __name__ == '__main__':
    paraData()