function Decoder(bytes, port) {
  var decoded = {};

  if (bytes.length == 9) { //Received GPS data
    decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
    decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign) {
      decoded.alt = 0xFFFF0000 | altValue;
    } else {
      decoded.alt = altValue;
    }
    decoded.hdop = bytes[8] / 10.0;
    decoded.valid_gps = true;
  } else {
    decoded.valid_gps = false;
  }
  return decoded;
}
