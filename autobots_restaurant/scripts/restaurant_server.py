#!/usr/bin/env python3
import rospy
import actionlib
from autobots_restaurant.srv import Order, OrderResponse, OrderItem
from autobots_restaurant.msg import OrderStatusAction, OrderStatusFeedback, OrderStatusResult

class RestaurantServer:
    def __init__(self):
        rospy.init_node('restaurant_server')
        self.order_service = rospy.Service('place_order', Order, self.handle_order)
        self.action_server = actionlib.SimpleActionServer('order_status', OrderStatusAction, self.handle_order_status, False)
        self.action_server.start()
        self.orders = {}
        self.order_counter = 0
        rospy.loginfo("✅ Restaurant server is ready!")

    def handle_order(self, req):
        try:
            order_id = f"order_{self.order_counter}"
            self.order_counter += 1
            
            total_price = sum(item.price for item in req.items)
            total_time = sum(self.calculate_preparation_time(item) for item in req.items)
            
            self.orders[order_id] = {
                'customer_name': req.customer_name,
                'items': req.items,
                'status': 'accepted',
                'progress': 0.0,
                'total_time': total_time,
                'time_remaining': total_time
            }
            
            rospy.loginfo(f"📦 New order: {order_id} from {req.customer_name}")
            
            return OrderResponse(
                success=True,
                message=f"Order {order_id} accepted",
                total_price=total_price,
                estimated_time=total_time
            )
            
        except Exception as e:
            return OrderResponse(success=False, message=str(e), total_price=0.0, estimated_time=0.0)

    def handle_order_status(self, goal):
        order_id = goal.order_id
        if order_id not in self.orders:
            self.action_server.set_aborted()
            return
        
        order = self.orders[order_id]
        rate = rospy.Rate(1)
        
        while order['time_remaining'] > 0:
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                return
            
            order['time_remaining'] -= 1
            order['progress'] = ((order['total_time'] - order['time_remaining']) / order['total_time']) * 100
            
            feedback = OrderStatusFeedback()
            feedback.status = self.get_status_message(order['progress'])
            feedback.progress_percentage = order['progress']
            feedback.time_remaining = order['time_remaining']
            
            self.action_server.publish_feedback(feedback)
            rate.sleep()
        
        result = OrderStatusResult(completed=True, final_status='delivered')
        self.action_server.set_succeeded(result)

    def calculate_preparation_time(self, item):
        base_time = 5.0
        size_factors = {'small': 0.8, 'medium': 1.0, 'large': 1.5}
        size_factor = size_factors.get(item.size.lower(), 1.0)
        extras_time = len(item.extras) * 1.0
        return base_time * size_factor + extras_time

    def get_status_message(self, progress):
        if progress < 25: return "Order received"
        elif progress < 50: return "Preparing"
        elif progress < 75: return "Cooking"
        elif progress < 90: return "Finalizing"
        else: return "Almost ready"

if __name__ == "__main__":
    server = RestaurantServer()
    rospy.spin()
