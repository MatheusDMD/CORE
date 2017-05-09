/* Extension demonstrating a blocking command block */
/* Sayamindu Dasgupta <sayamindu@media.mit.edu>, May 2014 */
var http = new XMLHttpRequest();
var btn_clicked = false; // This becomes true after the alarm goes off

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
      var url_btn = "http://127.0.0.1:8000/?led=status&ln="+btn;
      console.log(url_btn);
      http.onreadystatechange = function() {
        if (http.readyState === XMLHttpRequest.DONE) {
            console.log(http.responseText);
            if(http.responseText == 'Status 1'){
              btn_clicked = true;
            }
            callback();
       }
      };
      http.open("GET", url_btn, true);
      http.send();
      return true;
    };

    ext.when_btn_clicked = function() {
       if (btn_clicked === true) {
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
        status = "set";
      }else if (status == 'OFF'){
        status = "clear";
      }
      var url_led = "http://127.0.0.1:5000/?led="+status+"&ln="+led_number;
      console.log(url_led);
      http.open("GET", url_led, true);
      http.send();
    };

    // Block and block menu descriptions
    var descriptor = {
        blocks: [
            ['h', 'When button %m.btn_options clicked', 'when_btn_clicked'],
            ['w', 'get button %m.btn_options status', 'get_btn_status'],
            ['', 'Set LED %m.btn_options state to %m.led_status', 'toggle_led','1','ON'],
        ],
        menus: {
        led_status: ['OFF', 'ON'],
        btn_options: ['1', '2', '3'],
    }
    };

    // Register the extension
    ScratchExtensions.register('CORE Extension', descriptor, ext);
})();
