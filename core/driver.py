import sim

class Driver:
    _instance = None
    cid = -1
    M_objs = {}

    @classmethod
    def instance(cls):
        if not cls._instance:
            cls._instance = cls()
        return cls._instance

    def initConnection(self, ip='127.0.0.1', port=19999):
        sim.simxFinish(-1)
        print(ip, port)
        #self.cid = sim.simxStart(ip, port, True, True, 5000, 5)
        self.cid = sim.simxStart(ip,port,True,True,5000,5)
        if self.cid == -1:
            raise Exception('Fail: Connected to remote API server')
        return self.cid

    def get(self, obj_name):
        if self.M_objs.get(obj_name):
            return self.M_objs[obj_name]
        res, obj_id = sim.simxGetObjectHandle(self.cid, obj_name, sim.simx_opmode_oneshot_wait)
        if res != 0:
            raise Exception(f"Fail: connect obj {obj_name} failed")
        self.M_objs[obj_name] = obj_id
        return obj_id      

