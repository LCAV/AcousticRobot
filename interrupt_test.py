import time
import threading

shutdown_event = threading.Event()

def dowork():
	while not shutdown_event.is_set():
		print(time.time())
		time.sleep(1.0)

def main():
	t = threading.Thread(target=dowork,args=())
	t.start()
	print("Instance started")
	
	try:
		while t.isAlive:
			t.join(timeout=1.0) #timeout so that keyboard interrupts are detected
	except(KeyboardInterrupt,SystemExit):
		shutdown_event.set()
	pass

main()

