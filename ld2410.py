
"""
            LD2410 Driver in python
            =======================


            (c) Ed French 2022

            Threaded driver in python for the LD2410 mmWave radar module.

            Not suitable for micropython, but some of the code might be helpful!



"""
import logging
logging.basicConfig(level=logging.DEBUG)
import serial
from threading import Thread
from collections import deque
from datetime import datetime,timedelta
import time
import random



class Thresholds:
    def __init__(self,moving:list[int|None]|None,static:list[int|None]|None):
        """
                None or missing parameters indicates no threshold set at this range 
                Whole lists can be ommitted if no thresholds of that type are required
        """
        self.moving=moving
        self.static=static


class Reading:
    TARGET_STATES={0x00:"No target",
                    0x01:"Moving target",
                    0x02:"Static target",
                    0x03:"Moving & static target"}

    def __init__(self,
                    target_state:int,
                    moving_target_range:int,
                    moving_target_energy:int,
                    static_target_range:int,
                    static_target_energy:int,
                    detection_distance:int,
                    max_moving_distance:int,
                    max_static_distance:int,
                    moving:list[int],
                    static:list[int],
                    timestamp:datetime|None=None):
        """
            Stores a single reading from the sensor
        """
        if timestamp is None:
            timestamp=datetime.now()
        self.timestamp=timestamp
        self.moving:list[int]=moving
        self.static:list[int]=static
        self.target_state:int=target_state
        self.moving_target_range:int=moving_target_range
        self.moving_target_energy:int=moving_target_energy
        self.static_target_range:int=static_target_range
        self.static_target_energy:int=static_target_energy
        self.detection_distance:int=detection_distance
        self.max_moving_distance:int=max_moving_distance
        self.max_static_distance:int=max_static_distance

   

    def __str__(self)->str:
        return f"{self.timestamp}\tMoving: {self.moving}\tStatic: {self.static}"


    def __repr__(self)->str:
        return str(self)


    def compare(self,thresholds:Thresholds)->bool:
        """
            Tests if levels in this reading exceed the Thresholds
        """
        return False # Placeholder

    @classmethod
    def new_from_bytes(cls,seq:bytes)->object:
        movs:list[int]=[seq[11+i] for i in range(12)]
        stats:list[int]=[seq[23+i] for i in range(12)]
        res=cls(target_state=seq[0],
                    moving_target_range=byte_pair_to_int(seq[1:3]),
                    moving_target_energy=seq[3],
                    static_target_range=byte_pair_to_int(seq[4:6]),
                    static_target_energy=seq[6],
                    detection_distance=byte_pair_to_int(seq[7:9]),
                    max_moving_distance=seq[9],
                    max_static_distance=seq[10],
                    moving=movs,
                    static=stats
                )
        return res

        



class LD2410Exception(Exception):
    pass

def list_ints_to_bytes(inp:list[int])->bytes:
    res=b''
    for b in inp:
        res+=b.to_bytes(1,byteorder="little",signed=False)
    return res

def bytes_to_list_ints(inp:bytes)->list[int]:
    bits=[c for c in inp]
    return bits

def byte_pair_to_int(inp:bytes)->int:
    return int.from_bytes(inp,byteorder="little",signed=False)

def bytes_to_hex(inp:bytes)->str:
    digits:list[int]=bytes_to_list_ints(inp)
    items=[f"0x{i:02x}" for i in digits]
    return ",".join(items)

class LD2410(Thread):
    COMMAND_PREAMBLE:bytes=list_ints_to_bytes([0xFD,0xFC,0xFB,0xFA])
    COMMAND_POSTAMBLE:bytes=list_ints_to_bytes([0x04,0x03,0x02,0x01])
    REPORTING_PREAMBLE:bytes=list_ints_to_bytes([0xF4,0xF3,0xF2,0xF1])
    REPORTING_POSTAMBLE:bytes=list_ints_to_bytes([0xF8,0xF7,0xF6,0xF5])

    VALID_STATES=["startup","waiting_for_eng_ack","waiting_for_first_report","reporting"]



    def __init__(self,
                    port:str,
                    baud:int=256000,
                    maxlen:int=50,
                    thresholds:Thresholds=None):

        """

            maxlen is the maximum number of measurements kept in the buffer (deque)
            thresholds is a Threshold object for testing prescence


            .set_thresholds
            .get_latest
            .is_present
            
        
        """
        self.port=port
        self.baud=baud
        self.thresholds:Thresholds=thresholds # Default is no threshold set
        self.running:bool=False
        self.stopped:bool=True
        self.buffer:deque=deque(maxlen=maxlen)
        self.serial:serial.Serial|None=None
        self.state:str=""
        self.state_change_time:datetime=datetime.now()
        self.change_state("startup")
        
        

        Thread.__init__(self) # Complete thread initialisation
        self.daemon=True

    def read_bytes(self,length:int)->bytes:
        r:bytes=self.serial.read(length)
        #logging.debug(f"\t\t>>>> {bytes_to_hex(r)}")
        return r

    def change_state(self,new_state):
        if new_state not in self.VALID_STATES:
            raise LD2410Exception(f"Attempt to enter new state: {new_state} that isn't in VALID_STATES")

        logging.debug(f"LD2410 changing state from {self.state} to {new_state}")
        self.state=new_state
        self.state_change_time=datetime.now()


    def _tx_sequence(self,sequence:bytes|list[int]):

        # Convert a list of ints to a bytes obj if required
        if isinstance(sequence,bytes):
            payload:bytes=sequence
        else:
            payload:bytes=list_ints_to_bytes(sequence)

        #Preamble
        self.serial.write(self.COMMAND_PREAMBLE)
        #Length
        length:int=len(payload)
        length_b:bytes=length.to_bytes(2,byteorder="little")
        self.serial.write(length_b)
        #Payload
        self.serial.write(payload)
        #Postamble
        self.serial.write(self.COMMAND_POSTAMBLE)
        


    def _read_any_message(self):
        """
                Just reads the next message, returns a tuple
                (preamble:bytes,message:bytes,postamble:bytes
                does not check anything except for timeouts
        """
        # Wait until we have the first byte of a preamble
        #logging.debug("Fetching a response")
        preamble_target_no:int=0 #
        preamble:bytes=b"" 
        target_preamble:bytes=b'' # set according to the first byte spotted matching preamble

        while True:
            #print(".",end="")
            c_b:bytes=self.serial.read(1)
            #print("%",end="")
            if len(c_b)==0:
                logging.debug("timeout!")
            else:
                c:int=c_b[0]
                #print(f"!0x{c:02x},",end="")
                if preamble_target_no==0:
                    if (c==self.COMMAND_PREAMBLE[0] or c==self.REPORTING_PREAMBLE[0]):
                        # We found the first byte of preamble (maybe!)

                        # Figure out what to expect next
                        if c==self.REPORTING_PREAMBLE[0]: # First char sets the type
                            target_preamble=self.REPORTING_PREAMBLE
                        else:
                            target_preamble=self.COMMAND_PREAMBLE

                        # Move to next one as target!
                        preamble+=bytes([c])
                        preamble_target_no+=1
                        #print("\nFound first!\n")
                else:
                    # Must be second character here!
                    if not c==target_preamble[preamble_target_no]:
                        # Bad subsequent char, so restart
                        preamble=b""
                        target_preamble=b""
                        preamble_target_no=0
                        #print("Sequence broken\n\n")
                    else:
                        # We found the next good character
                        preamble_target_no+=1
                        #print(f"\nFound {preamble_target_no}th character in preamble")
                        preamble+=bytes([c])
                        if preamble_target_no==4:
                            # We finished finding the preamble
                            #print(f"\nPreamble Done OK")
                            break


        length_raw:bytes=self.read_bytes(2)
        if not len(length_raw)==2:
            raise LD2410Exception("Didn't get two bytes for the message length in the expected time")
        length:int=byte_pair_to_int(length_raw)
        #logging.debug(f"\t\t\t\tPayload detected is {length} bytes")


        payload:bytes=self.read_bytes(length)
        if not len(payload)==length:
            raise LD2410Exception(f"Payload was short, only received : {len(payload)} bytes")
        #logging.debug(f"\t\t\tPayload received: {bytes_to_hex(payload)}")


        #Read and check postamble
        postamble:bytes=self.read_bytes(4)
        if not len(postamble)==4:
            raise LD2410Exception(f"Failed to receive the full 4 byte postamble")
        #logging.debug("\t\t\t\t\tpostamble OK")
        return (preamble,payload,postamble)
        
    def _flush_serial(self):
        self.serial.reset_input_buffer()


    def _rx_sequence(self,report_expected:bool,
                        retries=10,
                        timeout=1)->bytes:
        """
            Fetches a response from the sensor

                report_expected
                         is True if we are waiting for a measurement report
                         or False if we want a command ack or response
                retries
                         is the number of times it will try again to 
                         get the desired type of response

                timeout
                        is the serial port timeout used
        
        """
        
        if report_expected:
            expected_preamble=self.REPORTING_PREAMBLE
            expected_postamble=self.REPORTING_POSTAMBLE
        else:
            expected_preamble=self.COMMAND_PREAMBLE
            expected_postamble=self.COMMAND_POSTAMBLE
        if self.serial is not None:
            self.serial.timeout=timeout
        
        for try_no in range(retries):
            preamble,payload,postamble=self._read_any_message()
            if preamble==expected_preamble and postamble==expected_postamble:
                return payload
            logging.debug("Ignoring a message with the wrong pre-post-ambles")

        raise LD2410Exception(f"Failed to get a message of type {'report' if report_expected else 'command ack'} after {retries} attempts")

    def _enter_configuration_mode(self):
        logging.debug("\n\n\n*********** Entering config mode")
        self._tx_sequence([0xFF,0x00,0x01,0x00])
        resp:bytes=self._rx_sequence(report_expected=False,retries=10,timeout=2)
        if not resp==list_ints_to_bytes([0xFF,0x01,0x00,0x00,0x01,0x00,0x40,0x00]):
            raise LD2410Exception("Failed to get ack to entering config mode")
        logging.debug("Successfully entered config mode")

    def _leave_configuration_mode(self):
        logging.debug("\n\n\n*********** Leaving configuration mode")
        self._tx_sequence([0xFE,0x00])
        resp:bytes=self._rx_sequence(report_expected=False,retries=5,timeout=2)
        if not resp==list_ints_to_bytes([0xFE,0x01,0x00,0x00]):
            raise LD2410Exception("Failed to get ack to leaving config mode")
        logging.debug("Successfully left config mode")


    def _request_firmware_version(self):
        
        logging.info("\n\n\n************ Requesting firmware version")
        self._tx_sequence([0xA0,0x00])
        resp:bytes=self._rx_sequence(report_expected=False,retries=3,timeout=2)
        logging.info(f"Firmare code found: {bytes_to_hex(resp)}")
        


    def _send_reset(self):
        logging.info("\n\n\n************ Sending reset")
        self._tx_sequence([0xA3,0x00])
        resp:bytes=self._rx_sequence(report_expected=False,retries=3,timeout=2)
        if not resp==list_ints_to_bytes([0xA3,0x01,0x00,0x00]):
            raise LD2410Exception("Failed to get an ack on the attempt to reset")
        logging.debug("Reset acknowledged")


    def _start_engineering_mode(self):
        logging.info("\n\n\n************ Entering engineering mode")
        self._tx_sequence([0x62,0x00])
        resp:bytes=self._rx_sequence(report_expected=False,retries=10,timeout=2)
        if not resp==list_ints_to_bytes([0x62,0x01,0x00,0x00]):
            raise LD2410Exception(f"Failed to get the correct ack for entering engineering mode, instead rx'ed : ")

        logging.debug("Successfully entered engineering mode (ack'd)")
        

    def __enter__(self):
        if not self.serial is None:
            raise serial.SerialException("Attempt to open a port via a context manager (with) that is already open")
        if not self.port == "test":
            self.serial=serial.Serial(port=self.port,
                                baudrate=self.baud,
                                bytesize=serial.EIGHTBITS,
                                stopbits=serial.STOPBITS_ONE)




            

        self.start() # Launch own thread

    


        return self


    def __exit__(self,exc_type, exc_val, exc_tb):
        # Stop thread
        if not self.running:
            raise LD2410Exception("Attempt to properly end the monitoring thread, but somehow it's already gone!")

        self.running=False
        timeout:datetime=datetime.now()+timedelta(seconds=20)
        while True:
            if datetime.now()>timeout:
                raise LD2410Exception("Failed to stop the thread inside the timeout")
            if self.stopped: # Shut down elegantly within the timeout :-)
                break
            time.sleep(0.1)


        self.serial.close()

    def _get_next_reading(self,timeout_s:float=2)->Reading:
        """
            Grabs the next reading from the sensor, returns it 
            raises LD2410Exception if no reading obtained within the timeout
        """
        # self._enter_configuration_mode()
        # self._start_engineering_mode()
        while True:
            # Deal with test mode
            if self.port=="test":
                time.sleep(2)
                movs:list[int]=[random.randrange(1,50) for _ in range(9)]
                stat:list[int]=[random.randrange(1,50) for _ in range(9)]
                return Reading(moving=movs,static=stat)

            # Take a proper reading!
            #logging.debug("getting a new reading")
            seq:bytes=self._rx_sequence(report_expected=True,retries=1,timeout=2)
            #logging.debug(f"Received payload back of : {bytes_to_hex(seq)}")
            if len(seq)==35:
                break

        return Reading.new_from_bytes(seq)

        

    def set_presence_thresholds(self,levels:Thresholds):
        self.thresholds=levels

    def _test_presence(self,reading:Reading)->bool:
        """
            tests if the latest reading indicates presence 
            when compared the the thresholds
        """
        return reading.compare(self.thresholds)

    def get_latest(self)->Reading:
        while len(self.buffer)==0:
            logging.debug("Waiting for buffer to be refilled")
            time.sleep(0.1)
        return self.buffer[-1]

    def run(self):
        self.running=True
        self.stopped=False
        logging.debug("Thread running")
                # # Just to test
        # temp:bytes=self.serial.read(1024)
        # print(bytes_to_hex(temp))
        # stop
        self._enter_configuration_mode()
        self._send_reset()
        logging.debug("brief pause")
        time.sleep(2)
        self._flush_serial()
        self._enter_configuration_mode()
        self._request_firmware_version()
        #self._leave_configuration_mode()

        self._start_engineering_mode()
        self._leave_configuration_mode()

        while (self.running):
            """
                    Main loop that gets readings back from the sensor
            """
            read:Reading=self._get_next_reading()
            #logging.debug("NEW READING IS RECORDED")
            self.buffer.append(read)
            #logging.debug("added to buffer ¬¬¬¬¬¬¬¬")
            if self.thresholds is not None:
                self.present=self._test_presence(read)

            

            

        self.stopped=True



def test_sensor():
    with LD2410("COM7",maxlen=20) as sensor:
        time.sleep(0)
        for _ in range(100):
            time.sleep(0.5)
            print(f"\n\nREADING SET\n{sensor.get_latest()}")


if __name__=="__main__":
    test_sensor()





