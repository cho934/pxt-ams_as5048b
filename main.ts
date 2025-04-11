/**
 * AS5048B Magnetic Rotary Encoder driver for MicroBit
 */

//% color=#0fbc11 icon="\uf085" block="AS5048B"
namespace AS5048B {
    // Constants for register addresses
    const REG_PROG = 0x03;          // Programming register
    const REG_ADDR = 0x15;          // Address register
    const REG_ZEROMSB = 0x16;       // Zero position MSB register
    const REG_ZEROLSB = 0x17;       // Zero position LSB register
    const REG_AGC = 0xFA;           // Automatic Gain Control register
    const REG_DIAG = 0xFB;          // Diagnostics register
    const REG_MAGNMSB = 0xFC;       // Magnitude MSB register
    const REG_MAGNLSB = 0xFD;       // Magnitude LSB register
    const REG_ANGLMSB = 0xFE;       // Angle MSB register
    const REG_ANGLLSB = 0xFF;       // Angle LSB register

    // Masks and constants
    const ANGLE_MASK = 0x3FFF;      // 14-bit mask for angle value
    const RESOLUTION = 16384;       // 2^14, maximum value for angle

    // Angle unit constants
    const U_RAW = 1;                // Raw sensor value (0-16383)
    const U_TRN = 2;                // Turn (0.0-1.0)
    const U_DEG = 3;                // Degrees (0.0-359.98)
    const U_RAD = 4;                // Radians (0.0-6.28)
    const U_GRAD = 5;               // Gradians (0.0-400.0)

    /**
     * Calculate the I2C address based on a1 and a2 pins
     * Equivalent to: #define AS5048B_ADDR(a1,a2)  (uint8_t)(0x40 | ( !a1 ? 0x2 : 0 ) | ( !a2 ? 0x1 : 0 ))
     * @param a1 Address pin A1 state
     * @param a2 Address pin A2 state
     * @returns Calculated I2C address
     */
    function calculateAddress(a1: number, a2: number): number {
        return 0x40 | (!a1 ? 0x2 : 0) | (!a2 ? 0x1 : 0);
    }

    /**
     * Creates a new AS5048B sensor instance
     * @param a1 State of A1 address pin (0 or 1)
     * @param a2 State of A2 address pin (0 or 1)
     */
    //% blockId=as5048b_create_with_pins block="créer capteur AS5048B avec pins a1 %a1 a2 %a2"
    //% a1.defl=1
    //% a2.defl=1
    //% weight=100
    export function createSensorWithPins(a1: number = 1, a2: number = 1): AS5048BSensor {
        const address = calculateAddress(a1, a2);
        return new AS5048BSensor(address);
    }

    /**
     * Creates a new AS5048B sensor instance
     * @param address I2C address of the sensor (default: 0x40)
     */
    //% blockId=as5048b_create block="créer capteur AS5048B à l'adresse %address"
    //% address.defl=0x40
    //% weight=99
    export function createSensor(address: number = 0x40): AS5048BSensor {
        return new AS5048BSensor(address);
    }

    /**
     * Class representing an AS5048B magnetic rotary encoder
     */
    export class AS5048BSensor {
        private i2cAddr: number;
        private clockWise: boolean;
        private lastAngleRaw: number;

        // Moving average variables
        private movingAvgEnabled: boolean;
        private movingAvgAlpha: number;
        private movingAvgSin: number;
        private movingAvgCos: number;
        private movingAvgAngle: number;
        private movingAvgCount: number;
        private readonly INIT_AVG_COUNT = 5;

        /**
         * Create a new AS5048B sensor
         * @param address I2C address of the sensor
         */
        constructor(address: number) {
            this.i2cAddr = address;
            this.clockWise = false;
            this.lastAngleRaw = 0;

            // Initialize moving average variables
            this.movingAvgEnabled = false;
            this.movingAvgAlpha = 0.2;
            this.movingAvgSin = 0;
            this.movingAvgCos = 0;
            this.movingAvgAngle = 0;
            this.movingAvgCount = 0;
        }

        // Rest of the class implementation remains unchanged

        /**
         * Set the I2C address for this sensor
         * @param address New I2C address
         */
        //% blockId=as5048b_set_address block="%sensor|définir l'adresse I2C à %address"
        //% weight=90
        setAddress(address: number): void {
            this.i2cAddr = address;
        }

        /**
         * Set rotation direction (clockwise or counter-clockwise)
         * @param cw true for clockwise, false for counter-clockwise
         */
        //% blockId=as5048b_set_direction block="%sensor|définir direction %cw"
        //% cw.defl=false
        //% weight=85
        setClockWise(cw: boolean): void {
            this.clockWise = cw;
            this.resetMovingAvg();
        }

        /**
         * Check if the sensor is connected
         * @returns true if connected, false otherwise
         */
        //% blockId=as5048b_is_connected block="%sensor|est connecté ?"
        //% weight=80
        isConnected(): boolean {
            try {
                // Read angle register to check connection
                pins.i2cWriteNumber(this.i2cAddr, REG_ANGLMSB, NumberFormat.UInt8BE);
                const result = pins.i2cReadNumber(this.i2cAddr, NumberFormat.UInt16BE);
                return true; // If we get here, communication worked
            } catch (e) {
                return false;
            }
        }

        /**
         * Read register value (8-bit)
         * @param reg Register address
         * @returns 8-bit register value
         */
        private readReg8(reg: number): number {
            pins.i2cWriteNumber(this.i2cAddr, reg, NumberFormat.UInt8BE);
            return pins.i2cReadNumber(this.i2cAddr, NumberFormat.UInt8BE);
        }

        /**
         * Read register value (16-bit)
         * @param reg Register address
         * @returns 14-bit register value (masked appropriately)
         */
        private readReg16(reg: number): number {
            pins.i2cWriteNumber(this.i2cAddr, reg, NumberFormat.UInt8BE);
            const value = pins.i2cReadNumber(this.i2cAddr, NumberFormat.UInt16BE);
            return value & ANGLE_MASK; // Mask to 14 bits
        }

        /**
         * Write register value (8-bit)
         * @param reg Register address
         * @param value Value to write
         */
        private writeReg(reg: number, value: number): void {
            pins.i2cWriteRegister(this.i2cAddr, reg, value);
        }

        /**
         * Get the raw angle value (0-16383)
         * @returns Raw angle value (14-bit)
         */
        //% blockId=as5048b_raw_angle block="%sensor|angle brut"
        //% weight=75
        getRawAngle(): number {
            let rawValue = this.readReg16(REG_ANGLMSB);

            // Handle clockwise rotation if needed
            if (this.clockWise) {
                rawValue = ANGLE_MASK - rawValue;
            }

            this.lastAngleRaw = rawValue;
            return rawValue;
        }

        /**
         * Get the angle in degrees (0-359.98)
         * @returns Angle in degrees
         */
        //% blockId=as5048b_angle_deg block="%sensor|angle en degrés"
        //% weight=70
        getAngle(): number {
            return this.convertAngle(U_DEG, this.getRawAngle());
        }

        /**
         * Get the angle in radians (0-6.28)
         * @returns Angle in radians
         */
        //% blockId=as5048b_angle_rad block="%sensor|angle en radians"
        //% weight=65
        getAngleRad(): number {
            return this.convertAngle(U_RAD, this.getRawAngle());
        }

        /**
         * Convert raw angle to specified unit
         * @param unit Unit to convert to (U_RAW, U_TRN, U_DEG, U_RAD, U_GRAD)
         * @param angle Raw angle value
         * @returns Converted angle value
         */
        private convertAngle(unit: number, angle: number): number {
            switch (unit) {
                case U_RAW:
                    return angle;
                case U_TRN:
                    return angle / RESOLUTION;
                case U_DEG:
                    return (angle * 360) / RESOLUTION;
                case U_RAD:
                    return (angle * 2 * Math.PI) / RESOLUTION;
                case U_GRAD:
                    return (angle * 400) / RESOLUTION;
                default:
                    return angle;
            }
        }

        /**
         * Get the Automatic Gain Control value
         * @returns AGC value (0-255)
         */
        //% blockId=as5048b_get_agc block="%sensor|valeur AGC"
        //% weight=60
        getAgc(): number {
            return this.readReg8(REG_AGC);
        }

        /**
         * Program the current position as the zero position
         */
        //% blockId=as5048b_program_zero block="%sensor|programmer position zéro"
        //% weight=55
        programZeroPosition(): void {
            // First write 0 to zero register
            this.zeroRegisterWrite(0);
            // Then read current angle and set it as zero position
            const newZero = this.readReg16(REG_ANGLMSB);
            this.zeroRegisterWrite(newZero);
        }

        /**
         * Write value to zero position register
         * @param value Zero position value
         */
        private zeroRegisterWrite(value: number): void {
            this.writeReg(REG_ZEROMSB, (value >> 6) & 0xFF);
            this.writeReg(REG_ZEROLSB, value & 0x3F);
        }

        /**
         * Read zero position register
         * @returns Zero position value
         */
        //% blockId=as5048b_read_zero_reg block="%sensor|lire registre zéro"
        //% weight=50
        zeroRegisterRead(): number {
            return this.readReg16(REG_ZEROMSB);
        }

        /**
         * Get the diagnostic value
         * @returns Diagnostic register value
         */
        //% blockId=as5048b_get_diag block="%sensor|valeur diagnostic"
        //% weight=45
        getDiagnostic(): number {
            return this.readReg8(REG_DIAG);
        }

        /**
         * Get the magnitude value of the magnetic field
         * @returns Magnitude value
         */
        //% blockId=as5048b_get_magnitude block="%sensor|valeur magnitude"
        //% weight=40
        getMagnitude(): number {
            return this.readReg16(REG_MAGNMSB);
        }

        /**
         * Check if there is a CORDIC overflow error
         * @returns true if CORDIC overflow error is present
         */
        //% blockId=as5048b_diag_cordic block="%sensor|cordic overflow ?"
        //% weight=35
        hasCordicOverflow(): boolean {
            const diag = this.getDiagnostic();
            return (diag & 0x0001) !== 0;
        }

        /**
         * Check if the magnet is too strong
         * @returns true if magnet is too strong
         */
        //% blockId=as5048b_diag_mag_high block="%sensor|aimant trop fort ?"
        //% weight=30
        isMagnetTooStrong(): boolean {
            const diag = this.getDiagnostic();
            return (diag & 0x0008) !== 0;
        }

        /**
         * Check if the magnet is too weak
         * @returns true if magnet is too weak
         */
        //% blockId=as5048b_diag_mag_low block="%sensor|aimant trop faible ?"
        //% weight=25
        isMagnetTooWeak(): boolean {
            const diag = this.getDiagnostic();
            return (diag & 0x0004) !== 0;
        }

        /**
         * Enable exponential moving average filtering
         * @param enable true to enable, false to disable
         * @param alpha Smoothing factor (0.0-1.0)
         */
        //% blockId=as5048b_enable_moving_avg block="%sensor|activer moyenne mobile %enable || alpha %alpha"
        //% enable.defl=true
        //% alpha.defl=0.2
        //% expandableArgumentMode="toggle"
        //% weight=20
        enableMovingAvg(enable: boolean, alpha: number = 0.2): void {
            this.movingAvgEnabled = enable;
            this.movingAvgAlpha = alpha < 0 ? 0 : (alpha > 1 ? 1 : alpha);
            this.resetMovingAvg();
        }

        /**
         * Reset moving average calculations
         */
        private resetMovingAvg(): void {
            this.movingAvgCount = 0;
            this.movingAvgSin = 0;
            this.movingAvgCos = 0;
            this.movingAvgAngle = 0;
        }

        /**
         * Update the exponential moving average
         * Should be called regularly to maintain accurate averaging
         */
        //% blockId=as5048b_update_moving_avg block="%sensor|mettre à jour moyenne mobile"
        //% weight=15
        updateMovingAvg(): void {
            if (!this.movingAvgEnabled) return;

            // Get angle in radians
            const angle = this.getAngleRad();
            const sin = Math.sin(angle);
            const cos = Math.cos(angle);

            // Initialize with simple average for first INIT_AVG_COUNT readings
            if (this.movingAvgCount < this.INIT_AVG_COUNT) {
                this.movingAvgSin += sin;
                this.movingAvgCos += cos;
                this.movingAvgCount++;

                if (this.movingAvgCount === this.INIT_AVG_COUNT) {
                    this.movingAvgSin /= this.INIT_AVG_COUNT;
                    this.movingAvgCos /= this.INIT_AVG_COUNT;
                }
                return;
            }

            // Apply exponential moving average
            this.movingAvgSin = this.movingAvgSin + this.movingAvgAlpha * (sin - this.movingAvgSin);
            this.movingAvgCos = this.movingAvgCos + this.movingAvgAlpha * (cos - this.movingAvgCos);

            // Calculate angle from sin and cos components
            let avgAngle = 0;
            if (this.movingAvgSin < 0) {
                avgAngle = 2 * Math.PI - Math.acos(this.movingAvgCos);
            } else {
                avgAngle = Math.acos(this.movingAvgCos);
            }

            // Convert back to raw format
            this.movingAvgAngle = (avgAngle / (2 * Math.PI)) * RESOLUTION;
        }

        /**
         * Get the moving average angle in specified unit
         * @param unit Unit to convert to (U_RAW, U_TRN, U_DEG, U_RAD, U_GRAD)
         * @returns Moving average angle in specified unit
         */
        //% blockId=as5048b_get_moving_avg block="%sensor|obtenir moyenne mobile || unité %unit"
        //% unit.defl=3
        //% expandableArgumentMode="toggle"
        //% weight=10
        getMovingAvg(unit: number = U_DEG): number {
            return this.convertAngle(unit, this.movingAvgAngle);
        }
    }

    // Export unit constants for external use
    export const Unit = {
        RAW: U_RAW,
        TURN: U_TRN,
        DEGREES: U_DEG,
        RADIANS: U_RAD,
        GRADIANS: U_GRAD
    };
}