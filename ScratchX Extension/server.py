from flask import Flask, request, Response
from xml.etree.ElementTree import *
app = Flask(__name__)

#main = Element('btn_status')
#main.text = "OFF"

global_status = ['OFF']*3

@app.route('/')
def hello_world():
    return 'Hello, World!'

@app.route('/led',methods=['GET'])
def led_status_change():
    global global_status
    status = int(request.args.get('st'))
    led_number = int(request.args.get('ln'))
    if status == 1:
        global_status[led_number-1] = 'ON'
    elif status == 0:
        global_status[led_number-1] = 'OFF'
    else:
        global_status[led_number-1] = 'None'
    print("LED is now:{0}".format(status))
    return "LED is now:{0}".format(status)

@app.route('/btn_status/<int:btn>',methods=['GET'])
def get_btn_status(btn):
    global global_status
    return global_status[btn-1]

@app.route('/btn/<int:btn>',methods=['GET'])
def get_btn_status_core(btn):
    global global_status
    value = "<current> <status id=1 value={0}> </status> </current>".format(global_status[0])
    return Response(value, mimetype='xml')


if __name__ == '__main__':
    app.run(host="0.0.0.0", threaded=True)
