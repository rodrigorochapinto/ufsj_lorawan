function encodeDownlink(input) {
  return {
    bytes: [input.data.led],
    fPort: input.fPort,
    warnings: [],
    errors: []
  };
}

function decodeDownlink(input) {
  return {
    data: {
      bytes: input.bytes
    },
    warnings: [],
    errors: []
  }
}