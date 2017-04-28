from flask import Flask, request
app = Flask(__name__)

global_status = ['OFF']*3

@app.route('/')
def hello_world():
    return 'Hello, World!'

@app.route('/led',methods=['POST'])
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
    return "LED is now:{0}".format(status)

@app.route('/btn_status/<int:btn>',methods=['GET'])
def get_btn_status(btn):
    global global_status
    return global_status[btn-1]
