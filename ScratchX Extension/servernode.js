// Load the http module to create an http server.
var express = require('express');
var app = express();
var http = require('http');

app.get('/', function (req, res) {
   res.send('Hello World');
})

app.get('/status', function (req, res) {
   res.send('1');
})

// Configure our HTTP server to respond with Hello World to all requests.
var server = http.createServer(function (request, response) {
  response.writeHead(200, {"Content-Type": "text/plain"});
  response.end("Hello World\n");
  console.log('Hello');
});

// Listen on port 8000, IP defaults to 127.0.0.1
server.listen(8000, '0.0.0.0');
