/*
 * mavlink-packer-v2.ts
 *
 * Copyright (c) 2019, 
 * Institute of Flight Mechanics and Control, University of Stuttgart.
 * Pascal Groß <pascal.gross@ifr.uni-stuttgart.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

import {MAVLinkPackerBase} from "../mavlink-packer-base";
import {MAVLinkMessage, writeInt64LE, writeUInt64LE} from "../mavlink-message";

export class MAVLinkPackerV2 extends MAVLinkPackerBase {
    protected start_marker: number = 0xFD;
    protected minimum_packet_length: number = 12;

    constructor() {
        super();
    }

    packMessage(message: MAVLinkMessage): Buffer {
        const buffer = Buffer.alloc(this.minimum_packet_length + message._payload_length);
        buffer.writeUInt8(this.start_marker, 0);
        buffer.writeUInt8(message._payload_length, 1);
        buffer.writeUInt8(0, 2);
        buffer.writeUInt8(0, 3);
        buffer.writeUInt8(0, 4);
        buffer.writeUInt8(message._system_id, 5);
        buffer.writeUInt8(message._component_id, 6);
        buffer.writeUIntLE(message._message_id, 7, 3);

        let start = 0;
        for (const field of message._message_fields) {
            const field_name: string = field[0];
            const field_type: string = field[1];
            const extension_field: boolean = field[2];
            const field_length = message.sizeof(field_type);
            if (!extension_field) {
                this.write(buffer, message[field_name], start + this.minimum_packet_length - 2, field_type);
                start += field_length;
            }
        }

        let actual = message.x25CRC(buffer.slice(1, this.minimum_packet_length + message._payload_length - 2));

        buffer.writeUInt16LE(actual, this.minimum_packet_length + message._payload_length - 2);
        return buffer;
    }

    private write(bytes: Buffer, message_field: any, start: number, type: string) {
        const words = type.split('[')
        const base_type = words[0]
        const base_size = MAVLinkMessage.sizeof(base_type)
        let array_length = 1
        if (words.length > 1) {
            array_length = parseInt(words[1].slice(0, -1)) // the slice is there to remove closing ']'
            if (base_type === 'char') { // char array is a string
                // need special handling because the string may be shorter than message field size
                const text = message_field as string
                let written = bytes.write(text, start, text.length, 'ascii')
                while (written < array_length) { // fill rest of the buffer with NULL
                    bytes.write('\0', start + written, 1, 'ascii')
                    written += 1
                }
            } else {
                for (let i = 0; i < array_length; i++) {
                    const offset = i * base_size + start
                    this.write_single(bytes, message_field[i], offset, base_type)
                }
            }
        } else {
            this.write_single(bytes, message_field, start, type)
        }
    }

    private write_single(bytes: Buffer, message_value: any, start: number, base_type: string): void {
        switch (base_type) {
            case "uint8_t":
                bytes.writeUInt8(message_value, start);
                break;
            case "uint16_t":
                bytes.writeUInt16LE(message_value, start);
                break;
            case "uint32_t":
                bytes.writeUInt32LE(message_value, start);
                break;
            case "uint64_t":
                writeUInt64LE(bytes, message_value, start);
                break;
            case "int8_t":
                bytes.writeInt8(message_value, start);
                break;
            case "int16_t":
                bytes.writeInt16LE(message_value, start);
                break;
            case "int32_t":
                bytes.writeInt32LE(message_value, start);
                break;
            case  "int64_t":
                writeInt64LE(bytes, message_value,start);
                break;
            case "float":
                bytes.writeFloatLE(message_value, start);
                break;
            case "double":
                bytes.writeDoubleLE(message_value, start);
                break;
            case "char":
                bytes.write(message_value, start, 1, 'ascii');
                break;
        }
    }
}
