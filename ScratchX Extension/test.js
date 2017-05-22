/* Extension demonstrating a blocking command block */
/* Sayamindu Dasgupta <sayamindu@media.mit.edu>, May 2014 */
var http = new XMLHttpRequest();
var btn_clicked = false; // This becomes true after the alarm goes off
var btn_clicked_id = 0;
new (function() {
    var ext = this;

    // Cleanup function when the extension is unloaded
    ext._shutdown = function() {};

    // Status reporting code
    // Use this to report missing hardware, plugin or unsupported browser
    ext._getStatus = function() {
        return {status: 2, msg: 'Ready'};
    };

    ext.get_btn_status = function(btn, callback) {
      //192.168.43.33
      var url_btn = "http://192.168.0.144:5000/btn_status/"+btn;
      console.log(url_btn);
      http.onreadystatechange = function() {
        if (http.readyState === XMLHttpRequest.DONE) {
            console.log(http.responseText);
            if(http.responseText == "1"){
              btn_clicked = true;
              btn_clicked_id = btn;
            }
            callback();
       }
      };
      http.open("GET", url_btn, true);
      http.send();
      return true;
    };

    ext.when_btn_clicked = function(btn) {
       if (btn_clicked === true  && btn_clicked_id === btn) {
           btn_clicked = false;
           return true;
       }
       return false;
     };

    // Functions for block with type 'w' will get a callback function as the
    // final argument. This should be called to indicate that the block can
    // stop waiting.
    ext.toggle_led = function(led_number, status) {
      if(status == 'ON'){
        status = "1";
      }else if (status == 'OFF'){
        status = "0";
      }
      var url_led = "http://192.168.0.144:5000/led?st="+status+"&ln="+led_number;
      console.log(url_led);
      http.open("GET", url_led, true);
      http.send();
    };

    // Block and block menu descriptions
    var descriptor = {
        blocks: [
            ['h', 'When button %m.btn_options clicked', 'when_btn_clicked',"0"],
            ['w', 'get button %m.btn_options status', 'get_btn_status'],
            ['', 'Set LED %m.btn_options state to %m.led_status', 'toggle_led','1','ON'],
        ],
        menus: {
        led_status: ['OFF', 'ON'],
        btn_options: ['0','1', '2', '3'],
    }
    };

    // Register the extension
    ScratchExtensions.register('CORE Extension', descriptor, ext);
})();
