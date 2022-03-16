import rclpy
import sys
from gazebo_msgs.srv import SpawnEntity
from transforms3d.euler import euler2quat

def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('spawn_entity')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    entity_model = sys.argv[1]
    spawn_pose = sys.argv[2:5]
    spawn_quat = euler2quat(float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))

    req = SpawnEntity.Request()
    req.name ='r2d2'
    req.xml = entity_model
    req.robot_namespace = '/r2d2'
    req.reference_frame = 'world'
    req.initial_pose.position.x = float(spawn_pose[0])
    req.initial_pose.position.y = float(spawn_pose[1])
    req.initial_pose.position.z = float(spawn_pose[2])
    req.initial_pose.orientation.x = spawn_quat[1]
    req.initial_pose.orientation.y = spawn_quat[2]
    req.initial_pose.orientation.z = spawn_quat[3]
    req.initial_pose.orientation.w = spawn_quat[0]

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(
            'Service not available, waiting for spawn_entity...'
        )
    
    res = cli.call_async(req)
    rclpy.spin_until_future_complete(node, res)

    if res.result() is not None:
        node.get_logger().info(
            'Result: ' + str(res.result().success) + '/Message: ' + res.result().status_message
        )
    else:
        node.get_logger().info(
            'Service call failed: %r' % (res.exception())
        )
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
