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
            const colorTable8888 = {};
            for (let y = this.height - 1; y >= 0; y--) { // Start from the last row and go up
                for (let x = 0; x < this.width; x++) {
                    const idx = (this.width * y + x) << 2; // Calculate RGBA index
                    const r = this.data[idx];
                    const g = this.data[idx + 1];
                    const b = this.data[idx + 2];
                    const a = this.data[idx + 3] > 0? 1 : 0;
                    const rgb565 = convertToRGB565(r, g, b);
                    const composed_color = (rgb565 << 8) | a; 
                    colorTable[composed_color] = composed_color;
                    colorTable8888[composed_color] = [r,g,b,a*255];
                    rgb565Array.push(composed_color);
                }
            }

            const color_array = Object.values(colorTable);
            const color_array_8888 = Object.values(colorTable8888);
            while (color_array.length < 16) {
              color_array.push(0);
              color_array_8888.push([0,0,0,0]);
            }

            
            // Create a new PNG object
            const color_table_png = new PNG({ width: 16, height:1 });

            // Copy the pixel data to the PNG data buffer
            for (let i = 0; i < color_array.length; i++) {
              color_table_png.data[i*4+0] = color_array_8888[i][0];
              color_table_png.data[i*4+1] = color_array_8888[i][1];
              color_table_png.data[i*4+2] = color_array_8888[i][2];
              color_table_png.data[i*4+3] = color_array_8888[i][3];
            }

            // Save the PNG to a file
            color_table_png.pack().pipe(fs.createWriteStream(outputPath)).on('finish', () => {
                console.log('PNG file has been saved!');
            });

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