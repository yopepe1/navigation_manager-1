import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue # msg for treating interfaces(params)
from rcl_interfaces.srv import GetParameters                            # srv to get parameters
from rcl_interfaces.srv import SetParameters                            # srv to set parameters     # you can write "from rcl_interfaces.srv import GetParameters, SetParameters"

class Bg_Param(Node):
    def __init__(self):
        super().__init__('cg_turtle')   # take over __init__ from Node class ... 'cg_turtle' is node_name

    def setParam(self, red, green, blue):
        client = self.create_client(
            SetParameters,
            '/turtlesim/set_parameters'#.format_map(locals())
            #'/turtlesim/turtlesim/set_parameters'#.format_map(locals())
        )

        ready = client.wait_for_service(timeout_sec=5.0)

        if not ready:
            raise RuntimeError('Wait for service timed out')    # generate runtime exception

        req_for_set = SetParameters.Request()

        # setting for red
        param = Parameter()                                 # prepare instance to access values
        param.name = "background_r"                         # name
        param.value.type = ParameterType.PARAMETER_INTEGER  # type
        param.value.integer_value = red                     # value
        req_for_set.parameters.append(param)                # add this to request_list to request

        # setting for green
        param = Parameter()                                 # prepare instance to access values
        param.name = "background_g"                         # name
        param.value.type = ParameterType.PARAMETER_INTEGER  # type
        param.value.integer_value = green                   # value
        req_for_set.parameters.append(param)                # add this to request_list to request

        # setting for blue
        param = Parameter()                                 # prepare instance to access values
        param.name = "background_b"                         # name
        param.value.type = ParameterType.PARAMETER_INTEGER  # type
        param.value.integer_value = blue                    # value
        req_for_set.parameters.append(param)                # add this to request_list to request

        future = client.call_async(req_for_set)             # call service with request Asynchronously

    def getParam(self):
        client = self.create_client(
            GetParameters,                          # type
            '/turtlesim/get_parameters'   # service name
            #'/turtlesim/turtlesim/get_parameters'   # service name
        )

        ready = client.wait_for_service(timeout_sec=5.0)

        if not ready:
            raise RuntimeError('Wait for service timed out')    # generate runtime exception

        req_for_get = GetParameters.Request()

        req_for_get.names = ["background_r", "background_g", "background_b"]    # set names for parameters to request

        future = client.call_async(req_for_get)         # call service with request Asynchronously
        rclpy.spin_until_future_complete(self, future)  # loop until receive the response

        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(                                 # display error message clearly
                'Exception while calling service of node'
                "'{args.node_name}':{e}".format_map(locals())
            )

        print('background_r:', response.values[0].integer_value)
        print('background_g:', response.values[1].integer_value)
        print('background_b:', response.values[2].integer_value)

def main(args=None):
    rclpy.init(args=args)

    param_client = Bg_Param()

    param_client.setParam(100, 10, 100)   # set red, green, blue
    param_client.getParam()             # get parameter

if __name__ == '__main__':
    main()

