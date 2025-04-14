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
//Inversion REG_DIAG / REG_AGC ???
    const REG_DIAG = 0xFA;          // Diagnostics register
    const REG_AGC = 0xFB;           // Automatic Gain Control register
    const REG_MAGNMSB = 0xFC;       // Magnitude MSB register
    const REG_MAGNLSB = 0xFD;       // Magnitude LSB register
    const REG_ANGLMSB = 0xFE;       // Angle MSB register
    const REG_ANGLLSB = 0xFF;       // Angle LSB register

    // Constants
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
                let result = this.getDiagnostic();
                if(result >= 1)
                    return true;
                else
                    return false;
                
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
            //let readBuffer = pins.i2cReadBuffer(this.i2cAddr, 1);
            //return readBuffer[0];
        }

        /**
         * Read register value (16-bit)
         * @param reg Register address
         * @returns 14-bit register value (masked appropriately)
        * (7..0 MSB + 5..0 LSB) => valeur de 14 bits
        */
        private readReg16(reg: number): number {
            // Nombre d'octets à lire
            const nbByte2Read = 2;
            let requestResult: number;
            let readArray: number[] = [0, 0];
            let readValue = 0;

            // Démarrer la transmission I2C
            pins.i2cWriteNumber(
                this.i2cAddr,
                reg,
                NumberFormat.UInt8LE,
                true  // répéter (équivalent à endTransmission(false))
            );

            // Vérification d'erreur I2C (simplifiée pour micro:bit)
            try {
                // Lire les données
                let readBuffer = pins.i2cReadBuffer(this.i2cAddr, nbByte2Read);

                // Convertir le buffer en tableau
                readArray[0] = readBuffer[0];
                readArray[1] = readBuffer[1];

                // Combiner les deux octets pour obtenir la valeur de 14 bits
                readValue = (readArray[0] << 6);
                readValue += (readArray[1] & 0x3F);

                return readValue;
            } catch (e) {
                // Gestion d'erreur I2C simplifiée
                serial.writeLine("readReg16 I2C error");
                return 0;
            }
        }

        /**
         * Write register value (8-bit)
         * @param reg Register address
         * @param value Value to write
         */
        private writeReg(reg: number, value: number): void {
            let buffer = pins.createBuffer(2);
            buffer.setUint8(0, reg);     // Adresse du registre
            buffer.setUint8(1, value);   // Valeur à écrire
            pins.i2cWriteBuffer(this.i2cAddr, buffer);
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
                // 0b11111111111111 correspond à 16383 en décimal (0x3FFF) (14 bits à 1)
                rawValue = 0b11111111111111 - rawValue;
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
         * Get the Automatic Gain Control value (distance of the magnet)
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







/*
__Howto__
input.onButtonPressed(Button.A, function () {
    magencoders.stop()
})
let magencoders: MagEncoders.MagEncoders = null
serial.redirectToUSB()
magencoders = MagEncoders.createMagEncoder(true, false, true)
magencoders.start()
loops.everyInterval(50, function () {
    magencoders.getValues()
    serial.writeNumbers([magencoders.getLeftTotalCount(), magencoders.getRightTotalCount()])
})
basic.forever(function () {
})
*/

//% color=#000000 icon="\uf085" block="MAGENCODERS"
namespace MagEncoders {
    /**
     * Creates a new MagEncoder instance for dual AS5048B encoders
     * @param is1EncoderRight If the first encoder is connected to the right wheel
     * @param invertEncoderRight Invert right encoder direction
     * @param invertEncoderLeft Invert left encoder direction
     */
    //% blockId=as5048b_create_magencoders block="créer encodeurs avec (A1=1,A2=1)=droite %is1EncoderRight|inverser droite %invertRight|inverser gauche %invertLeft"
    //% is1EncoderRight.defl=true
    //% invertRight.defl=false
    //% invertLeft.defl=false
    //% weight=98
    export function createMagEncoder(
        is1EncoderRight: boolean = true,
        invertRight: boolean = false,
        invertLeft: boolean = false
    ): MagEncoders {
        return new MagEncoders(is1EncoderRight, invertRight, invertLeft);
    }


    /**
     * Class for managing dual magnetic rotary encoders (AS5048B)
     * Used for differential drive robots with left and right encoders
     */
    //% color=#000000
    //% blockId=as5048b_magencoders_class
    //% block="encodeurs magnétiques"
    export class MagEncoders {
        private sensor1: AS5048B.AS5048BSensor;
        private sensor2: AS5048B.AS5048BSensor;
        private is1EncoderRight: boolean;
        private invertEncoderR: boolean;
        private invertEncoderL: boolean;

        private encoder1Previous: number;
        private encoder2Previous: number;
        
        public encoderRSum: number;
        public encoderLSum: number;

        public deltaEncoderRightFiltered: number;
        public deltaEncoderLeftFiltered: number;

        /**
         * Constructor for magnetic encoders
         * @param is1EncoderRight If the first encoder is on the right wheel
         * @param invertEncoderRight Invert right encoder direction
         * @param invertEncoderLeft Invert left encoder direction
         */
        constructor(
            is1EncoderRight: boolean = true,
            invertEncoderRight: boolean = false,
            invertEncoderLeft: boolean = false
        ) {
            // Configure AS5048B sensors with proper addresses
            // First sensor (A1=1, A2=1) => Address 0x40
            // Second sensor (A1=0, A2=1) => Address 0x42
            this.sensor1 = AS5048B.createSensorWithPins(1, 1);
            this.sensor2 = AS5048B.createSensorWithPins(0, 1);

            this.is1EncoderRight = is1EncoderRight;
            this.invertEncoderR = invertEncoderRight;
            this.invertEncoderL = invertEncoderLeft;

            this.encoder1Previous = 0;
            this.encoder2Previous = 0;
            this.encoderRSum = 0;
            this.encoderLSum = 0;

            this.deltaEncoderRightFiltered = 0; //right encoder delta values
            this.deltaEncoderLeftFiltered = 0; //left encoder delta values
        }

        /**
         * Start encoders and initialize counters
         */
        //% blockId=as5048b_magencoders_start block="%encoders|démarrer"
        //% weight=100
        public start(): void {
            this.encoder1Previous = ((this.sensor1.getRawAngle() - 8192.0) * 4.0);
            this.encoder2Previous = ((this.sensor2.getRawAngle() - 8192.0) * 4.0);
            this.encoderRSum = 0;
            this.encoderLSum = 0;
        }

        /**
         * Reset encoders and counters
         */
        //% blockId=as5048b_magencoders_stop block="%encoders|réinitialiser"
        //% weight=95
        public stop(): void {
            this.start(); // Reset values
        }


        private toInt16(value: number): number {
            //return (((value | 0) << 16) >> 16);
            // Garder seulement les 16 bits de poids faible
            let intValue = value & 0xFFFF;

            // Si le bit 15 est à 1, c'est un nombre négatif
            if (intValue & 0x8000) {
                return intValue - 0x10000; // Soustraire 2^16 pour obtenir la valeur négative correcte
            }
            return intValue;
        }

        /**
         * Get encoder values (delta movement since last reading)
         * @returns Object with right and left encoder delta values
         */
        //% blockId=as5048b_magencoders_get_values block="%encoders|lire valeurs"
        //% weight=90
        public getValues(): void {
            let deltaEncoderRight = 0.0;
            let deltaEncoderLeft = 0.0;

            this.deltaEncoderRightFiltered=0;
            this.deltaEncoderLeftFiltered=0;

            //utilisation du depassement d'un int16
            //[0;16383] -8192 * 4 = [-32768;32764]
            const encoder1 = (this.sensor1.getRawAngle() - 8192.0) * 4.0;
            const encoder2 = (this.sensor2.getRawAngle() - 8192.0) * 4.0;

            //debug
            //serial.writeValue("getRawAngle1", this.sensor1.getRawAngle());
            //serial.writeValue("encoder1", encoder1);
            
    

            // Determine deltas based on configuration
            if (this.is1EncoderRight) {
                deltaEncoderRight = encoder1 - this.encoder1Previous;
                deltaEncoderLeft = encoder2 - this.encoder2Previous;
            } else {
                deltaEncoderRight = encoder2 - this.encoder2Previous;
                deltaEncoderLeft = encoder1 - this.encoder1Previous;
            }
            
            //debug
            //serial.writeValue("deltaEncoderRight", deltaEncoderRight);
            
            
            // Invert if necessary
            if (this.invertEncoderR)
                deltaEncoderRight = -deltaEncoderRight;
            if (this.invertEncoderL)
                deltaEncoderLeft = -deltaEncoderLeft;

            // Convert to normalized values
            //deltaEncoderRight = this.toInt16(Math.floor(deltaEncoderRight)) / 4.0;
            //deltaEncoderLeft = this.toInt16(Math.floor(deltaEncoderLeft)) / 4.0;
            deltaEncoderRight = this.toInt16(deltaEncoderRight) / 4.0;
            deltaEncoderLeft = this.toInt16(deltaEncoderLeft) / 4.0;

            //debug
            //serial.writeValue("deltaEncoderRight final", deltaEncoderRight);

            // Update sums
            //this.encoderRSum += Math.floor(deltaEncoderRight);
            //this.encoderLSum += Math.floor(deltaEncoderLeft);
            this.encoderRSum += deltaEncoderRight;
            this.encoderLSum += deltaEncoderLeft;

            // Save previous values
            this.encoder1Previous = encoder1;
            this.encoder2Previous = encoder2;

            
            this.deltaEncoderRightFiltered=deltaEncoderRight;
            this.deltaEncoderLeftFiltered=deltaEncoderLeft;
            
        }

        /**
         * Get right encoder value
         */
        //% blockId=as5048b_magencoders_get_right block="%encoders|valeur delta encodeur droit"
        //% weight=85
        public getDeltaRightValue(): number {
            return this.deltaEncoderRightFiltered;
        }

        /**
         * Get left encoder value
         */
        //% blockId=as5048b_magencoders_get_left block="%encoders|valeur delta encodeur gauche"
        //% weight=84
        public getDeltaLeftValue(): number {
            return this.deltaEncoderLeftFiltered;
        }

        /**
         * Get total right encoder count
         */
        //% blockId=as5048b_magencoders_get_total_right block="%encoders|compteur total droit"
        //% weight=75
        public getRightTotalCount(): number {
            return this.encoderRSum;
        }

        /**
         * Get total left encoder count
         */
        //% blockId=as5048b_magencoders_get_total_left block="%encoders|compteur total gauche"
        //% weight=74
        public getLeftTotalCount(): number {
            return this.encoderLSum;
        }

        /**
         * Reset total encoder counts
         */
        //% blockId=as5048b_magencoders_reset_total block="%encoders|réinitialiser compteurs"
        //% weight=70
        public resetTotalCount(): void {
            this.encoderRSum = 0;
            this.encoderLSum = 0;
        }

        /**
         * Check if both encoders are connected
         */
        //% blockId=as5048b_magencoders_check block="%encoders|encodeurs connectés ?"
        //% weight=65
        public areEncodersConnected(): boolean {
            return this.sensor1.isConnected() && this.sensor2.isConnected();
        }

        /**
         * Check if encoder1 is connected
         */
        //% blockId=as5048b_magencoder1_check block="%encoders|encodeur1 connecté ?"
        //% weight=66
        public getEncoder1Connected(): boolean {
            return this.sensor1.isConnected();
        }

        /**
         * Check if encoder2 is connected
         */
        //% blockId=as5048b_magencoder2_check block="%encoders|encodeur2 connecté ?"
        //% weight=67
        public getEncoder2Connected(): boolean {
            return this.sensor2.isConnected();
        }
    }
}








/* 
__Howto__
// Initialisation avec vos paramètres
odometry.initialize(150, 2000); // 150mm entre les roues, 2000 ticks par mètre

// Dans votre boucle principale ou gestionnaire d'événements
basic.forever(function() {
    // Obtenir les valeurs delta des encodeurs (à adapter selon votre système)
    let rightDelta = getEncoderRightDelta();
    let leftDelta = getEncoderLeftDelta();

    // Mettre à jour l'odométrie
    odometry.update(rightDelta, leftDelta);

    // Afficher les informations si nécessaire
    basic.showString("X:" + odometry.X + " Y:" + odometry.Y);
})
*/
// Odometry module for MicroBit
// Calculates position and orientation using wheel encoder data
namespace odometry {
    // Global variables for position tracking
    export let X = 0;           // X position in mm
    export let Y = 0;           // Y position in mm  
    export let alphaRad = 0;       // Orientation angle in radians

    // Constants - adjust these based on your robot's specifications
    export let entraxeInMM = 100;  // Distance between wheels in mm
    export let ticksPerMeter = 2000;   // Number of ticks per meter

    // Variables to store encoder deltas
    let dRight = 0;
    let dLeft = 0;

    /**
     * Initialize the odometry module with specific parameters
     * @param wheelbase Distance between wheels in mm
     * @param ticksPerMeter Number of encoder ticks per meter
     */
    //% block="initialize odometry with wheelbase %wheelbase|mm and %ticksPerMeter|ticks per meter"
    export function initialize(entraxe_mm: number, nbticksPerMeter: number) {
        // Convert wheelbase from mm to ticks
        odometry.entraxeInMM = entraxe_mm ;//* nbticksPerMeter / 1000;
        odometry.ticksPerMeter = nbticksPerMeter;
        X = 0;
        Y = 0;
        alphaRad = 0;
    }

    /**
     * Reset position and orientation to zero
     */
    //% block="reset odometry"
    export function reset() {
        X = 0;
        Y = 0;
        alphaRad = 0;
    }

    /**
     * Set position and orientation to specific values
     * @param x X position in mm
     * @param y Y position in mm
     * @param angle Orientation in radians
     */
    //% block="set position to x: %x|y: %y|angle: %angle"
    export function setPosition(x: number, y: number, anglerad: number) {
        X = x;
        Y = y;
        alphaRad = anglerad;
    }

    /**
     * Update odometry with new encoder values
     * @param rightDelta Right encoder delta in ticks
     * @param leftDelta Left encoder delta in ticks
     */
    /*
    //% block="update with right delta: %rightDelta|left delta: %leftDelta"
    export function update(leftDelta: number, rightDelta: number) {
        dRight = rightDelta;
        dLeft = leftDelta;

        // Calculate angle and distance variations
        let dAlpha = (dRight - dLeft) / 2;         // Variation of the angle in radians
        let dDelta = (dRight + dLeft) / 2;         // Variation of the forward movement

        // Convert to radians and update angle
        alphaRad += dAlpha / entraxe_tick;

        // Keep alpha within [-π, π] range
        while (alphaRad > Math.PI) {
            alphaRad -= 2 * Math.PI;
        }
        while (alphaRad < -Math.PI) {
            alphaRad += 2 * Math.PI;
        }

        // Calculate position offsets
        let dX = Math.cos(alphaRad) * dDelta;
        let dY = Math.sin(alphaRad) * dDelta;

        // Update position in mm
        X += dX * 1000 / ticksPerMeter;  // Convert from ticks to mm
        Y += dY * 1000 / ticksPerMeter;  // Convert from ticks to mm
    }*/

    /**
     * Update odometry with new encoder values in mm
     * @param rightDeltaMm Right encoder delta in mm
     * @param leftDeltaMm Left encoder delta in mm
     */
    //% block="update with right delta: %rightDeltaMm|mm left delta: %leftDeltaMm|mm"
    export function update(rightDeltaMm: number, leftDeltaMm: number) {
        // Calculate distance traveled and angle variation
        let deltaDist = (leftDeltaMm + rightDeltaMm) / 2;
        let diffCount = rightDeltaMm - leftDeltaMm;
        let deltaTheta = diffCount / entraxeInMM; // In radians

        if (diffCount == 0) {
            // Consider movement as a straight line
            // Update position
            X += deltaDist * Math.cos(alphaRad);
            Y += deltaDist * Math.sin(alphaRad);
        } else {
            // Approximate by considering that the robot follows an arc
            // Calculate the radius of curvature of the circle
            let R = deltaDist / deltaTheta;

            // Update position
            X += R * (-Math.sin(alphaRad) + Math.sin(alphaRad + deltaTheta));
            Y += R * (Math.cos(alphaRad) - Math.cos(alphaRad + deltaTheta));

            // Update heading
            alphaRad += deltaTheta;

            // Limit heading to +/- PI to be able to turn in both directions
            if (alphaRad > Math.PI) {
                alphaRad -= 2 * Math.PI;
            } else if (alphaRad <= -Math.PI) {
                alphaRad += 2 * Math.PI;
            }
        }
    }

    /**
     * Update odometry with new encoder values in ticks
     * @param rightDeltaTicks Right encoder delta in ticks
     * @param leftDeltaTicks Left encoder delta in ticks
     */
    //% block="update with right delta: %rightDeltaTicks|ticks left delta: %leftDeltaTicks|ticks"
    export function updateFromTicks(rightDeltaTicks: number, leftDeltaTicks: number) {
        // Convert ticks to mm
        let rightDeltaMm = rightDeltaTicks * 1000 / ticksPerMeter;
        let leftDeltaMm = leftDeltaTicks * 1000 / ticksPerMeter;

        // Call the regular update function
        update(rightDeltaMm, leftDeltaMm);
    }

    /**
     * Get current X position in float mm
     */
    //% block="get X position (float mm)"
    export function getX(): number {
        return X;
    }

    /**
     * Get current X position round in mm
     */
    //% block="get X position (integer mm)"
    export function getXint(): number {
        return Math.floor(X);
    }

    /**
     * Get current Y position in float mm
     */
    //% block="get Y position (float mm)"
    export function getY(): number {
        return Y;
    }

    /**
     * Get current Y position round in mm
     */
    //% block="get Y position (integer mm)"
    export function getYint(): number {
        return Math.floor(Y);
    }

    /**
     * Get current orientation in radians
     */
    //% block="get orientation (radians)"
    export function getOrientationRad(): number {
        return alphaRad;
    }

    /**
     * Get current orientation in degrees
     */
    //% block="get orientation (degrees)"
    export function getOrientationDegrees(): number {
        return alphaRad * 180 / Math.PI;
    }

    /**
     * Calculate distance to a point
     * @param x X coordinate of the target point
     * @param y Y coordinate of the target point
     */
    //% block="distance to point x: %x|y: %y"
    export function distanceTo(x: number, y: number): number {
        let dx = x - X;
        let dy = y - Y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate angle in radians to a point (relative to current orientation)
     * @param x X coordinate of the target point
     * @param y Y coordinate of the target point
     */
    //% block="angle to point x: %x|y: %y"
    export function angleTo(x: number, y: number): number {
        let dx = x - X;
        let dy = y - Y;
        let targetAngle = Math.atan2(dy, dx);

        // Calculate the difference and normalize to [-π, π]
        let angleDiff = targetAngle - alphaRad;
        while (angleDiff > Math.PI) {
            angleDiff -= 2 * Math.PI;
        }
        while (angleDiff < -Math.PI) {
            angleDiff += 2 * Math.PI;
        }

        return angleDiff;
    }
}