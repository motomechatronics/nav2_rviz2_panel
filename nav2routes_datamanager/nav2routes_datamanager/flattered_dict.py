


def flatten_dict(dictionary):
    flattened = {}
    for key, value in dictionary.items():
        if isinstance(value, dict):
            nested = flatten_dict(value)
            for nested_key, nested_value in nested.items():
                flattened[f"{key}.{nested_key}"] = nested_value
        else:
            flattened[key] = value
    return flattened


ros2_dictionary = {
    'node': {
        'name': 'my_node',
        'namespace': '/my_namespace',
        'parameters': {
            'param1': 10,
            'param2': 'Hello, ROS2!'
        },
        'topics': {
            'topic1': {
                'type': 'std_msgs.msg.String',
                'publisher': True,
                'subscriber': True
            },
            'topic2': {
                'type': 'sensor_msgs.msg.Imu',
                'publisher': True,
                'subscriber': False
            }
        },
        'services': {
            'service1': {
                'type': 'example_interfaces.srv.AddTwoInts',
                'server': True,
                'client': True
            },
            'service2': {
                'type': 'example_interfaces.srv.SetBool',
                'server': True,
                'client': False
            }
        }
    },
    'launch': {
        'package': 'my_package',
        'executable': 'my_executable',
        'arguments': ['--param1', 'value1', '--param2', 'value2']
    }
}

dict = flatten_dict(ros2_dictionary)
keys = dict.keys()
values = dict.values()
print(keys)
print('*********')
print(values)