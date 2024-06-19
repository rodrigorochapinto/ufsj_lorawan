
function Decoder(bytes, port) {

  function bytesToFloat(bytes) {
    var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
    var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
    var e = bits>>>23 & 0xff;
    var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
    var f = sign * m * Math.pow(2, e - 150);
    return parseFloat(f.toFixed(2));
  }  
    return bits * sign;
  }	
  
  return {
    temperatura: 	bytesToFloat(bytes.slice(0, 4)),
	  umidade: 		  bytesToFloat(bytes.slice(4, 8)),
	  altitude: 		bytesToFloat(bytes.slice(8, 12)),
	  pressao: 		  bytesToFloat(bytes.slice(12, 16))
  };
}