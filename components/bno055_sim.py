from hal_impl.i2c_helpers import I2CSimBase

class BNO055Sim(I2CSimBase):
    def i2CTransaction(self, port, device_address, data_to_send, send_size, data_received, receive_size):
        '''
            To give data back use ``data_received``::
            
                data_received[:] = [1,2,3...]
            
            :returns: number of bytes returned
        '''
        return receive_size
