const fs = require('fs');
const { PNG } = require('pngjs');

function convertToRGB565(r, g, b) {
    // Convert 8-bit RGB to 16-bit RGB565
    const r5 = (r >> 3) & 0x1F; // 5 bits for red
    const g6 = (g >> 2) & 0x3F; // 6 bits for green
    const b5 = (b >> 3) & 0x1F; // 5 bits for blue
    return (r5 << 11) | (g6 << 5) | b5;
}

function processImage(inputPath, outputPath) {
    fs.createReadStream(inputPath)
        .pipe(new PNG())
        .on('parsed', function () {
            const rgb565Array = [];
            const colorTable = {};
            for (let y = this.height - 1; y >= 0; y--) { // Start from the last row and go up
                for (let x = 0; x < this.width; x++) {
                    const idx = (this.width * y + x) << 2; // Calculate RGBA index
                    const r = this.data[idx];
                    const g = this.data[idx + 1];
                    const b = this.data[idx + 2];
                    const rgb565 = convertToRGB565(r, g, b);
                    colorTable[rgb565] = rgb565;
                    rgb565Array.push(rgb565);
                }
            }

            const pixel_array = [];
            const color_array = Object.values(colorTable);
            while (color_array.length < 16) {
              color_array.push(0);
            }
            
            for(let i=0; i< rgb565Array.length; i+=2)
            {
                const index0 = color_array.indexOf(rgb565Array[i]);
                const index1 = color_array.indexOf(rgb565Array[i+1]);
                console.log(index0, index1)

                console.log(((index0 << 4)  | index1).toString(2).padStart(8, '0'))
                pixel_array.push((index0 << 4)  | index1);
            }
            console.log(colorTable)

            // Write the RGB565 data to a binary file
            const buffer = Buffer.alloc(pixel_array.length + color_array.length*2);
            
            for(let i=0; i< color_array.length; i++)
            {
              buffer.writeUint16BE(color_array[i], i*2);
            }

            console.log(color_array)
            for (let i = 0; i < pixel_array.length; i++) 
            {
              buffer.writeUint8(pixel_array[i], i + color_array.length*2);
            }

            fs.writeFileSync(outputPath, buffer);
            console.log(`RGB565 data written to ${outputPath}`);
        })
        .on('error', (err) => {
            console.error(`Error processing the image: ${err.message}`);
        });
}

// Get command-line arguments
const args = process.argv.slice(2);
if (args.length < 2) {
    console.error('Usage: node script.js <inputFile> <outputFile>');
    process.exit(1);
}

const [inputPng, outputFile] = args;
processImage(inputPng, outputFile);