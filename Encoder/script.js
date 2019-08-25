const inputCmd = document.getElementById('cmd');
const inputId = document.getElementById('id');
const inputData = document.getElementById('frameData');
const outputEncoded = document.getElementById('encoded');
const outputEncodedCRC = document.getElementById('encodedCRC');
const buttonCrcReq = document.getElementById('buttonCrcReq');
const add0x = document.getElementById('add0x');
let prefix = '0x';
const Mask = (e, reg, length) => {
    let text = e.target.value;

    return {
        filterHex() {
            if (!e.data) return this;
            if (!e.data.match(reg) || e.target.value.length > length) {
                text = e.target.value.slice(0, -1);
            }
            return this;
        },
        separateNumbers() {
            text = text.split(',').reduce((acc, val) => {
                const num = parseInt(val);
                if (!isNaN(num)) {
                    acc += `${num},`;
                    return acc;
                }
                else {
                    return acc;
                }
            }, '');
            text = text.length > 1 ? text.slice(0, -1) : text;
            return this;
        },
        getText: () => text
    }
};
const Filter = (text) => ({
    add0x: () => text.split(' ').reduce((acc, val) => (acc += ' 0x' + val), '').trim(),
    remove0x: () => text.replace(/0x/gi, '')
});
add0x.onchange = (e) => {
    if (e.target.checked) {
        prefix = '0x';
        outputEncoded.text = Filter(outputEncoded.text).add0x();
        outputEncodedCRC.text = Filter(outputEncodedCRC.text).add0x();
    }
    else {
        prefix = '';
        outputEncoded.text = Filter(outputEncoded.text).remove0x();
        outputEncodedCRC.text = Filter(outputEncodedCRC.text).remove0x();
    }
};
inputCmd.oninput = (e) => {
    let frame = outputEncoded.text.split(' ');
    e.target.value = Mask(e, /[0-F]/i, 4).filterHex().getText();
    switch (e.target.value.length) {
        case 2:
            frame[0] = prefix + e.target.value.slice(0, 2);
            break;
        case 4:
            frame[1] = prefix + e.target.value.slice(2);
            break;
        default:
            return;
    }
    outputEncoded.text = frame.join(' ');
};
inputData.onchange = (e) => {
    let frame = outputEncoded.text.split(' ');
    e.target.value = Mask(e, /(\d|,)/, 23).separateNumbers().getText();
    const dataArray = e.target.value.split(',').map(val => {
        const num = parseInt(val, 10);
        if (num > 65535) return 65535;
        if (num < 0) return 0;
        return num;
    });
    const hexArray = dataArray.map(v => v.toString(16).length % 2 ? '0' + v.toString(16) : v.toString(16));
    for (let i = 0; i < 12; i++) {
        hexArray[i] = hexArray[i] || '00'
    }
    let dataString = hexArray.reduce((acc, val) => {
        const arr = val.match(/.{1,2}/g);
        let str ='0x00'+arr.reduce((acc, val) => {
            acc = acc + ' 0x' + val.toUpperCase();
            return acc
        }, '');
        let sl=str.slice(-10).trim();
        return acc += ' ' +sl;
    }, '');
    dataString = add0x.checked ? dataString : Filter(dataString).remove0x();
    outputEncoded.text = outputEncoded.text.slice(0, 20) + dataString.trim();
};
inputId.onchange = (e) => {
    let frame = outputEncoded.text.split(' ');
    e.target.value = Mask(e, /(\d|,)/, 23).separateNumbers().getText();
    const idArray = e.target.value.split(',');
    let lowByte = 0;
    let highByte = 0;
    for (let id of idArray) {
        if (id < 8) {
            lowByte |= 1 << id;
        }
        else {
            highByte |= 1 << (id - 8);
        }
    }
    frame[2] = prefix + `0${highByte.toString(16).toUpperCase()}`.slice(-2)
    frame[3] = prefix + `0${lowByte.toString(16).toUpperCase()}`.slice(-2)
    outputEncoded.text = frame.join(' ');
};
const makeCRCTable = function () {
    let c;
    let crcTable = [];
    for (let n = 0; n < 256; n++) {
        c = n;
        for (let k = 0; k < 8; k++) {
            c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
        }
        crcTable[n] = c;
    }
    return crcTable;
};

const crc32_calc = function (seq) {
    let crcTable = window.crcTable || (window.crcTable = makeCRCTable());
    let crc = 0 ^ (-1);

    for (let i = 0; i < seq.length; i++) {
        crc = (crc >>> 8) ^ crcTable[(crc ^ seq[i]) & 0xFF];
    }

    return (crc ^ (-1)) >>> 0;
};
buttonCrcReq.onclick = () => {
    let crc32 = crc32_calc(Filter(outputEncoded.text).remove0x().split(' ').map(val => parseInt(val, 16)))
        .toString(16).toUpperCase();
    crc32 = ('0' + crc32).slice(-8).match(/.{1,2}/g);
    outputEncodedCRC.text = outputEncoded.text
        + ` ${prefix}` + crc32[0]
        + ` ${prefix}` + crc32[1]
        + ` ${prefix}` + crc32[2]
        + ` ${prefix}` + crc32[3]

};