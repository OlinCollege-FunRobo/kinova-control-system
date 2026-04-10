from backend.kinova import Kinova
import sys, time
import numpy as np
import queue, threading

class Main:
        
    def __init__(self, loop_rate = 20) -> None:
        self.kinova_robot = Kinova()
        self.LOOP_RATE = 1 / float(loop_rate)
        
        self.action_queue = queue.Queue()
        
        self.is_running = True
        
        self.start()
        
        self.background_thread = threading.Thread(target=self._start_loop, daemon=True)
        self.background_thread.start()
        print("loop Loop Started")
        
    # DO NOT TOUCH
    def _start_loop(self):
        try:
            while self.is_running:
                if not self.action_queue.empty():
                    func, args = self.action_queue.get()
                    print(f'Executing: {func.__name__}')
                    func(*args)
                self.loop()
                time.sleep(self.LOOP_RATE)
        except Exception as e:
            print(f'ERROR Background loop crashed: {e}')
            
            
    # DO NOT TOUCH
    def shutdown(self):
        print("Shutting down gracefully")
        self.is_running = False
        self.kinova_robot.set_torque(True)
        self.kinova_robot.stop()
        sys.exit(0)
            
    def start(self):
        self.home = False
        pass        
        
    def loop(self):        
        is_7DOF = None
        
        if(is_7DOF is None):
            raise ValueError("If you are using the big robot set is_7DOF to true. If you are using the small robot set is_7DOF to false")
        
        if(is_7DOF):
            HOME_POSITION = [
                np.float64(3.0973061488745386), 
                np.float64(5.986872949549664), 
                np.float64(3.3493788734243917), 
                np.float64(2.5405110702884017), 
                np.float64(0.13820194012813908), 
                np.float64(4.921791206368749), 
                np.float64(1.4136101676718038)
            ]
            next_position = [
                np.float64(2.67433864349041), 
                np.float64(5.472201622557529), 
                np.float64(2.848439071328814), 
                np.float64(1.9227318664263773), 
                np.float64(0.13880221663784292), 
                np.float64(4.921789075839876), 
                np.float64(1.4136101676718038)
            ]
            
        else:
            HOME_POSITION = [
                np.radians(100),
                np.radians(330),
                np.radians(125),
                np.radians(140),
                np.radians(260),
                np.radians(0),
            ]
            next_position = [
                np.radians(45),
                np.radians(350),
                np.radians(85),
                np.radians(80),
                np.radians(350),
                np.radians(90),
            ]
            
        if(self.home):
            self.kinova_robot.set_joint_angles(next_position)
            self.home = False
            # Wait 3 seconds for the move to actually complete
            time.sleep(3)
            
        if(not self.home):
            self.kinova_robot.set_joint_angles(HOME_POSITION)
            self.home = True
            # Wait 3 seconds for the move to actually complete
            time.sleep(3)
            

if __name__ == "__main__":
    final_project = Main()
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        final_project.shutdown()