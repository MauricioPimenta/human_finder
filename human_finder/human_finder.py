def main():
    print('Hi from human_finder.')
    # using ros2 jazzy, subscribe to tf transform to check if the transform with child frame id as "detected_human" is being published
    import rclpy
    import tf2_ros

    rclpy.init()
    node = rclpy.create_node('human_finder_node')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = node.create_rate(1.0)
    while rclpy.ok():
        try:
            # Check if the transform exists
            tf_buffer.lookup_transform("base_link", "detected_human", rclpy.Time(0), rclpy.Duration(seconds=1.0))
            print("Transform from base_link to detected_human is available.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Transform from base_link to detected_human is not available.")
        rate.sleep()


if __name__ == '__main__':
    main()
