#!/usr/bin/env python3
import rospy
import actionlib
from autobots_restaurant.srv import Order, OrderItem
from autobots_restaurant.msg import OrderStatusAction, OrderStatusGoal

class CustomerClient:
    def __init__(self):
        rospy.init_node('customer_client')
        rospy.wait_for_service('place_order')
        self.order_service = rospy.ServiceProxy('place_order', Order)
        self.action_client = actionlib.SimpleActionClient('order_status', OrderStatusAction)
        self.action_client.wait_for_server()
        rospy.loginfo("✅ Customer client is ready!")

    def place_order(self, customer_name, items, order_type):
        try:
            order_request = Order()
            order_request.customer_name = customer_name
            order_request.items = items
            order_request.order_type = order_type
            
            response = self.order_service(order_request)
            
            if response.success:
                rospy.loginfo(f"✅ Order placed! Total: ${response.total_price:.2f}, Time: {response.estimated_time:.1f}min")
                self.track_order_status("order_0")
            else:
                rospy.logerr(f"❌ Failed: {response.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Service call failed: {str(e)}")

    def track_order_status(self, order_id):
        goal = OrderStatusGoal(order_id=order_id)
        self.action_client.send_goal(goal)
        
        while not self.action_client.wait_for_result(rospy.Duration(1.0)):
            feedback = self.action_client.get_feedback()
            rospy.loginfo(f"📊 Status: {feedback.status} - {feedback.progress_percentage:.1f}% - ⏱️{feedback.time_remaining:.1f}min")
        
        result = self.action_client.get_result()
        if result.completed:
            rospy.loginfo(f"🎉 Order {result.final_status}!")

def create_order_item(name, category, size, extras, price):
    item = OrderItem()
    item.name = name
    item.category = category
    item.size = size
    item.extras = extras
    item.price = price
    return item

if __name__ == "__main__":
    client = CustomerClient()
    items = [
        create_order_item("Energon Pizza", "main", "large", ["extra cheese"], 15.99),
        create_order_item("Energon Drink", "drink", "medium", [], 3.99)
    ]
    client.place_order("Optimus Prime", items, "delivery")
