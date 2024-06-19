
function Decoder(bytes, port) {

  function bytesToFloat(bytes) {
    var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
    var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
    var e = bits>>>23 & 0xff;
    var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
    var f = sign * m * Math.pow(2, e - 150);
    return parseFloat(f.toFixed(2));
  }  
  
  function bytesToShort(bytes, signed) {
    var bits = bytes[0] | (bytes[1] << 8);
    var sign = 1;

    if (signed && bits >>> 15 === 1) {
      sign = -1;
      bits = bits & 0x7FFF;
    }

    return bits * sign;
  }	
  
  return {
    temperatura: bytesToFloat(bytes.slice(0, 4)),
	  umidade:     bytesToFloat(bytes.slice(4, 8)),
	  altitude:    bytesToShort(bytes.slice(8, 10))
  };
}