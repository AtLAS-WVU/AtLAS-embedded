import sensors
import imageprocessing
import cruising
import landing
import takeoff
import demo

line = ""

while line != 'quit':
    line = input("Hit enter to get the current number. ")
    print(demo.demo.get_number())

demo.demo.stop()