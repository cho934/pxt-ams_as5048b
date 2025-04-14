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

    // Constants
    const RESOLUTION = 16384;       // 2^14, maximum value for angle

    // Angle unit constants
    const U_RAW = 1;                // Raw sensor value (0-16383)
    const U_TRN = 2;                // Turn (0.0-1.0)
    const U_DEG = 3;                // Degrees (0.0-359.98)
    const U_RAD = 4;                // Radians (0.0-6.28)
    const U_GRAD = 5;               // Gradians (0.0-400.0)

    /**
     * Structure pour stocker les données et l'état de diagnostic de l'encodeur
     */
    export interface AS5048BData {
        rawAngle: number;      // Angle brut (0-16383)
        angle: number;         // Angle en degrés (0-359.98)
        magnitude: number;     // Magnitude du champ magnétique
        agc: number;           // Valeur de contrôle de gain automatique
        diagnostics: number;   // Valeur du registre de diagnostic
        isValid: boolean;      // Indique si les données sont valides
        errorType: string;     // Type d'erreur si les données ne sont pas valides
    }

    /**
     * Calculate the I2C address based on a1 and a2 pins
     * Equivalent to: #define AS5048B_ADDR(a1,a2)  (uint8_t)(0x40 | ( !a1 ? 0x2 : 0 ) | ( !a2 ? 0x1 : 0 ))
     * @param a1 Address pin A1 state
     * @param a2 Address pin A2 state
     * @returns Calculated I2C address
     */
    function calculateAddress(a1: boolean, a2: boolean): number {
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
    export function createSensorWithPins(a1: boolean = true, a2: boolean = true): AS5048BSensor {
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

        // Méthode de débogage pour comparer 8-bit vs 16-bit
        private debugReadReg8(reg: number): void {
            // Utiliser les deux méthodes et comparer
            let value8 = this.readReg8(reg);

            // Utiliser readReg16 pour obtenir un octet
            pins.i2cWriteNumber(this.i2cAddr, reg, NumberFormat.UInt8BE);
            let buffer = pins.i2cReadBuffer(this.i2cAddr, 1);
            let value16Method = buffer[0];

            pins.i2cWriteNumber(this.i2cAddr, reg, NumberFormat.UInt8BE);
            let buffer2 =  pins.i2cReadNumber(this.i2cAddr, NumberFormat.UInt8BE);

            serial.writeLine("Reg " + reg + " - readReg8: " + value8 + ", readReg16 method: " + value16Method + ", readReg8BE method: " + buffer2);
        }


        /**
         * Read register value (8-bit)
         * @param reg Register address
         * @returns 8-bit register value
         */
        private readReg8(reg: number): number {
            
            try {
                pins.i2cWriteNumber(this.i2cAddr, reg, NumberFormat.UInt8BE);
                return pins.i2cReadNumber(this.i2cAddr, NumberFormat.UInt8LE);
                //let readBuffer = pins.i2cReadBuffer(this.i2cAddr, 1);
                //return readBuffer[0];
            } catch (e) {
                // Gestion d'erreur I2C simplifiée
                serial.writeLine("readReg8 I2C error: " + e.message);
                return 0;
            }
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
                serial.writeLine("readReg16 I2C error: " + e.message);
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
             * Lit l'angle brut et les informations de diagnostic en une seule requête I2C
             * Lit les registres 0xFA à 0xFF en une seule opération
             * @returns Objet contenant l'angle et les informations de diagnostic
             */
        //% blockId=as5048b_raw_angle_with_diag block="%sensor|angle brut avec diagnostic"
        //% weight=74
        getRawAngleWithDiag(): AS5048BData {
            try {
                // Commencer la lecture à partir du registre 0xFA (REG_DIAG)
                pins.i2cWriteNumber(this.i2cAddr, REG_DIAG, NumberFormat.UInt8BE);

                // Lire les 6 octets consécutifs (de 0xFA à 0xFF)
                let buffer = pins.i2cReadBuffer(this.i2cAddr, 6);

                // Extraire les données du buffer
                const agc = buffer[0];           // REG_AGC (0xFA)
                const diag = buffer[1];          // REG_DIAG (0xFB)
                const magnMsb = buffer[2];       // REG_MAGNMSB (0xFC)
                const magnLsb = buffer[3];       // REG_MAGNLSB (0xFD)
                const angleMsb = buffer[4];      // REG_ANGLMSB (0xFE)
                const angleLsb = buffer[5];      // REG_ANGLLSB (0xFF)

                // Combiner les octets pour obtenir l'angle brut (14 bits)
                let rawAngle = (angleMsb << 6) + (angleLsb & 0x3F);

                // Gérer la rotation dans le sens horaire si nécessaire
                if (this.clockWise) {
                    rawAngle = 0x3FFF - rawAngle; // 0x3FFF = 16383 (14 bits à 1)
                }

                // Combiner les octets pour obtenir la magnitude
                const magnitude = (magnMsb << 6) + (magnLsb & 0x3F);

                // Vérifier la validité des données à partir du registre de diagnostic
                let isValid = true;
                let errorType = "";

                if ((diag & 0x0001) !== 1) {
                    isValid = false;
                    errorType = "OFFSET COMPENSATION NOT FINISHED";
                }else if ((diag & 0x0010) !== 0) {
                    isValid = false;
                    errorType = "CORDIC_OVERFLOW";
                } else if ((diag & 0x0100) !== 0) {
                    isValid = false;
                    errorType = "MAGNET_TOO_WEAK";
                } else if ((diag & 0x1000) !== 0) {
                    isValid = false;
                    errorType = "MAGNET_TOO_STRONG";
                }

                // Calculer l'angle en degrés
                const angleDeg = (rawAngle * 360) / RESOLUTION;

                // Stocker la valeur brute comme dernière valeur d'angle connue
                this.lastAngleRaw = rawAngle;

                /*
                //debug
                serial.writeLine("rawAngle=" + rawAngle);
                serial.writeLine("agc=" + agc);
                serial.writeLine("diag=" + diag);
                serial.writeLine("magnitude=" + magnitude);
                serial.writeLine("isValid=" + isValid);
                serial.writeLine("errorType=" + errorType);
                */
                
                // Retourner toutes les données dans un objet
                return {
                    rawAngle: rawAngle,
                    angle: angleDeg,
                    magnitude: magnitude,
                    agc: agc,
                    diagnostics: diag,
                    isValid: isValid,
                    errorType: errorType
                };
            } catch (e) {
                // En cas d'erreur I2C, retourner un objet avec des données invalides
                //TODO en cas d'error prendre la derniere valeur
                serial.writeLine("Erreur I2C dans getRawAngleWithDiag: " + e.message);
                return {
                    rawAngle: 0,
                    angle: 0,
                    magnitude: 0,
                    agc: 0,
                    diagnostics: 0,
                    isValid: false,
                    errorType: "I2C_ERROR"
                };
            }
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
                rawValue = 0x3FFF - rawValue;
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
            //this.debugReadReg8(REG_AGC);
            return this.readReg8(REG_DIAG); //TODO INVERSION CAUSE A DEBUGGER
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
            //this.debugReadReg8(REG_DIAG);
            return this.readReg8(REG_AGC);//TODO INVERSION CAUSE A DEBUGGER
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
            return (diag & 0x0010) !== 0;
        }

        /**
         * Check if the magnet is too strong
         * @returns true if magnet is too strong
         */
        //% blockId=as5048b_diag_mag_high block="%sensor|aimant trop fort ?"
        //% weight=30
        isMagnetTooStrong(): boolean {
            const diag = this.getDiagnostic();
            return (diag & 0x1000) !== 0;
        }

        /**
         * Check if the magnet is too weak
         * @returns true if magnet is too weak
         */
        //% blockId=as5048b_diag_mag_low block="%sensor|aimant trop faible ?"
        //% weight=25
        isMagnetTooWeak(): boolean {
            const diag = this.getDiagnostic();
            return (diag & 0x0100) !== 0;
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
    //% blockId=as5048b_create_magencoders block="créer encodeurs avec Sensor1=droite %is1EncoderRight|inverser droite %invertRight|inverser gauche %invertLeft|sensor1 A1-A2 %setsensor1_A1 %setsensor1_A2|sensor2 A1-A2 %setsensor2_A1 %setsensor2_A2"
    //% is1EncoderRight.defl=true
    //% invertRight.defl=false
    //% invertLeft.defl=false
    //% setsensor1_A1.defl=true
    //% setsensor1_A2.defl=true
    //% setsensor2_A1.defl=true
    //% setsensor2_A2.defl=false
    //% weight=98
    export function createMagEncoder(
        is1EncoderRight: boolean = true,
        invertRight: boolean = false,
        invertLeft: boolean = false,
        setsensor1_A1: boolean = true,
        setsensor1_A2: boolean = true,
        setsensor2_A1: boolean = true,
        setsensor2_A2: boolean = false
    ): MagEncoders {
        return new MagEncoders(is1EncoderRight, invertRight, invertLeft, setsensor1_A1, setsensor1_A2, setsensor2_A1, setsensor2_A2);
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
            invertEncoderLeft: boolean = false,
            setsensor1_A1: boolean = true,
            setsensor1_A2: boolean = true,
            setsensor2_A1: boolean = true,
            setsensor2_A2: boolean = false
        ) {
            // Configure AS5048B sensors with proper addresses
            // First sensor (A1=1, A2=1) => Address 0x40
            // Second sensor (A1=0, A2=1) => Address 0x42
            this.sensor1 = AS5048B.createSensorWithPins(setsensor1_A1, setsensor1_A2);
            this.sensor2 = AS5048B.createSensorWithPins(setsensor2_A1, setsensor2_A2);

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
            let data1 = this.sensor1.getRawAngleWithDiag();
            if (data1.isValid) {
                this.encoder1Previous = ((data1.rawAngle - 8192.0) * 4.0);
            }
            let data2 = this.sensor2.getRawAngleWithDiag();
            if (data2.isValid) {
                this.encoder2Previous = ((data2.rawAngle - 8192.0) * 4.0);
            }
            
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

            let data1 = this.sensor1.getRawAngleWithDiag();
            let data2 = this.sensor2.getRawAngleWithDiag();

            //utilisation du depassement d'un int16
            //[0;16383] -8192 * 4 = [-32768;32764]
            //const encoder1 = (this.sensor1.getRawAngle() - 8192.0) * 4.0;
            //const encoder2 = (this.sensor2.getRawAngle() - 8192.0) * 4.0;
            let encoder1 = 0;
            let encoder2 = 0;
            if (data1.isValid)
            {
                encoder1 = (data1.rawAngle - 8192.0) * 4.0;
            } else {
                // Gérer l'erreur
                serial.writeString("Error:data1: " + data1.errorType + "\n");
            }
            if (data2.isValid) {
                encoder2 = (data2.rawAngle - 8192.0) * 4.0;
            } else {
                // Gérer l'erreur
                serial.writeString("Error:data2: " + data2.errorType + "\n");
            }

            //debug
            //serial.writeValue("getRawAngle1", this.sensor1.getRawAngle());
            //serial.writeValue("encoder1", encoder1);
            
    

            // Determine deltas based on configuration
            if (this.is1EncoderRight) {
                if (data1.isValid) {
                    deltaEncoderRight = encoder1 - this.encoder1Previous;
                }
                if (data2.isValid) {
                    deltaEncoderLeft = encoder2 - this.encoder2Previous;
                }
            } else {
                if (data2.isValid) {
                    deltaEncoderRight = encoder2 - this.encoder2Previous;
                }
                if (data1.isValid) {
                    deltaEncoderLeft = encoder1 - this.encoder1Previous;
                }
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
     * @param trackWidth Distance between encoders' wheels in mm
     * @param ticksPerMeter Number of encoder ticks per meter
     */
    //% block="initialize odometry with trackWidth %trackWidth|mm and %ticksPerMeter|ticks per meter"
    export function initialize(entraxe_mm: number, nbticksPerMeter: number) {
        // Convert trackWidth from mm to ticks
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
    //% block="update with leftDelta: %leftDeltaMm|mm + rightDelta: %rightDeltaMm|mm "
    export function update(leftDeltaMm: number, rightDeltaMm: number) {
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
    //% block="update with leftDelta: %leftDeltaTicks|ticks + rightDelta: %rightDeltaTicks|ticks"
    export function updateFromTicks(leftDeltaTicks: number, rightDeltaTicks: number) {
        // Convert ticks to mm
        let rightDeltaMm = rightDeltaTicks * 1000 / ticksPerMeter;
        let leftDeltaMm = leftDeltaTicks * 1000 / ticksPerMeter;

        // Call the regular update function
        update(leftDeltaMm, rightDeltaMm);
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




/**
 * Namespace pour le contrôle du déplacement d'un robot microbit
 * Utilise l'odométrie et les encodeurs magnétiques pour le positionnement précis
 * Avec support de différents drivers moteurs via des callbacks
 */
namespace RobotMovement {
    // Définition des types pour les callbacks
    export type MotorDriverFunction = (leftSpeed: number, rightSpeed: number, leftDirection: Direction, rightDirection: Direction) => void;
    export type EncoderReaderFunction = () => { leftDelta: number, rightDelta: number };

    // Enum pour la direction des moteurs
    export enum Direction {
        Forward = 0,  // CW
        Backward = 1  // CCW
    }

    // Variables pour les moteurs et encodeurs
    let isInitialized = false;
    let baseSpeed = 50; // Vitesse de base des moteurs (0-100)

    // Callbacks pour les fonctions adaptables
    let motorDriverCallback: MotorDriverFunction = null;
    let encoderReaderCallback: EncoderReaderFunction = null;

    // Constantes PID
    const Kp = 0.5;    // Coefficient proportionnel
    const Ki = 0.05;   // Coefficient intégral
    const Kd = 0.2;    // Coefficient dérivé

    // Variables pour garder une trace de l'erreur précédente et de l'erreur cumulée
    let previousError = 0;
    let integralError = 0;

    // Timeout pour les mouvements (ms)
    const MOVEMENT_TIMEOUT = 10000;

    /**
     * Initialise le système de déplacement avec des callbacks adaptables
     * @param motorDriver Fonction pour contrôler les moteurs
     * @param encoderReader Fonction pour lire les valeurs des encodeurs
     */
    //% block="initialize %MotorDriverFunction %EncoderReaderFunction"
    export function initialize(motorDriver: MotorDriverFunction, encoderReader: EncoderReaderFunction): void {
        motorDriverCallback = motorDriver;
        encoderReaderCallback = encoderReader;
        isInitialized = true;

        // Vérification de l'initialisation
        if (motorDriverCallback && encoderReaderCallback) {
            basic.showIcon(IconNames.Yes);
            serial.writeLine("Système de déplacement initialisé");
        } else {
            basic.showIcon(IconNames.No);
            serial.writeLine("Erreur: callbacks non définis");
            isInitialized = false;
        }
    }

    /**
     * Définit la vitesse de base des moteurs
     * @param speed Vitesse entre 0 et 100
     */
    export function setBaseSpeed(speed: number): void {
        baseSpeed = Math.constrain(speed, 0, 100);
    }

    /**
     * Applique les commandes de moteur en utilisant le callback défini
     * @param leftSpeed Vitesse du moteur gauche (0-100)
     * @param rightSpeed Vitesse du moteur droit (0-100)
     * @param leftDir Direction du moteur gauche
     * @param rightDir Direction du moteur droit
     */
    function applyMotorCommand(leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction): void {
        if (motorDriverCallback) {
            // Limiter les vitesses entre 0 et 100
            leftSpeed = Math.constrain(leftSpeed, 0, 100);
            rightSpeed = Math.constrain(rightSpeed, 0, 100);

            // Appeler le callback moteur
            motorDriverCallback(leftSpeed, rightSpeed, leftDir, rightDir);
        }
    }

    /**
     * Lit les valeurs des encodeurs en utilisant le callback défini
     * @returns Objet contenant les deltas des encodeurs gauche et droit
     */
    function readEncoders(): { leftDelta: number, rightDelta: number } {
        if (encoderReaderCallback) {
            return encoderReaderCallback();
        }
        return { leftDelta: 0, rightDelta: 0 };
    }

    /**
     * Arrête les moteurs
     */
    //% block="stopMotors"
    export function stopMotors(): void {
        if (motorDriverCallback) {
            motorDriverCallback(0, 0, Direction.Forward, Direction.Forward);
        }
    }

    /**
     * Déplace le robot vers une position relative (x,y) en mm
     * @param distX distance à parcourir en x (mm)
     * @param distY distance à parcourir en y (mm)
     */
    //% block="movexy %distX %distY"
    export function movexy(distX: number, distY: number): void {
        if (!isInitialized) {
            serial.writeLine("Erreur: système de déplacement non initialisé");
            return;
        }

        // Calcul de la distance totale à parcourir
        const distance = Math.sqrt(distX * distX + distY * distY);

        // Calcul de l'angle cible en radians
        let targetAngle = Math.atan2(distY, distX);

        // Angle de rotation nécessaire (différence entre l'angle cible et l'angle actuel)
        let angleDiff = targetAngle - odometry.getOrientationRad();

        // Normalisation entre -π et π
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

        // Conversion en degrés pour la fonction turn
        let angleDiffDeg = angleDiff * 180 / Math.PI;

        // 1. Tourner vers la direction cible
        turn(angleDiffDeg);

        // 2. Avancer de la distance calculée
        moveForward(distance);
    }

    /**
     * Tourne le robot d'un angle spécifié en degrés
     * @param angleDegrees angle de rotation en degrés (positif = sens anti-horaire)
     */
    //% block="turn %angleDegrees"
    export function turn(angleDegrees: number): void {
        if (!isInitialized) {
            serial.writeLine("Erreur: système de déplacement non initialisé");
            return;
        }

        // Normaliser l'angle entre -180 et 180 degrés pour prendre le chemin le plus court
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;

        // Convertir en radians
        const angleRadians = angleDegrees * Math.PI / 180;

        // Position cible
        const targetOrientation = odometry.getOrientationRad() + angleRadians;

        // Réinitialiser les erreurs PID
        previousError = 0;
        integralError = 0;

        // Démarrer les moteurs dans la bonne direction
        if (angleDegrees > 0) {
            // Tourner à gauche (sens anti-horaire)
            applyMotorCommand(baseSpeed, baseSpeed, Direction.Backward, Direction.Forward);
        } else {
            // Tourner à droite (sens horaire)
            applyMotorCommand(baseSpeed, baseSpeed, Direction.Forward, Direction.Backward);
        }

        // Durée maximale pour éviter une boucle infinie
        const startTime = input.runningTime();

        // Boucle jusqu'à atteindre l'orientation cible
        while (true) {
            // Mettre à jour l'odométrie
            const encoderData = readEncoders();
            odometry.updateFromTicks(encoderData.leftDelta, encoderData.rightDelta);

            // Calculer l'erreur actuelle (différence entre l'orientation cible et l'orientation actuelle)
            let currentOrientation = odometry.getOrientationRad();
            let error = angleBetween(targetOrientation, currentOrientation);

            // Vérifier si l'objectif est atteint
            if (Math.abs(error) < 0.03) { // ~1.7 degrés de précision
                break;
            }

            // Calculer les composantes PID
            let proportional = error * Kp;
            integralError += error;
            let integral = integralError * Ki;
            let derivative = (error - previousError) * Kd;
            previousError = error;

            // Calculer la correction de la vitesse
            let correction = proportional + integral + derivative;

            // Appliquer la correction aux moteurs
            let leftSpeed = baseSpeed + correction;
            let rightSpeed = baseSpeed - correction;

            // Limiter les vitesses
            leftSpeed = Math.constrain(leftSpeed, 0, 100);
            rightSpeed = Math.constrain(rightSpeed, 0, 100);

            // Actualiser les vitesses des moteurs
            if (angleDegrees > 0) {
                // Tourner à gauche
                applyMotorCommand(leftSpeed, rightSpeed, Direction.Backward, Direction.Forward);
            } else {
                // Tourner à droite
                applyMotorCommand(leftSpeed, rightSpeed, Direction.Forward, Direction.Backward);
            }

            // Vérifier le timeout
            if (input.runningTime() - startTime > MOVEMENT_TIMEOUT) {
                serial.writeLine("Timeout de rotation");
                break;
            }

            // Petit délai pour éviter une utilisation trop intensive du processeur
            basic.pause(10);
        }

        // Arrêter les moteurs
        stopMotors();
    }

    /**
     * Déplace le robot vers l'avant d'une distance spécifiée en mm
     * @param distance distance à parcourir en mm
     */
    //% block="moveForward %distance"
    function moveForward(distance: number): void {
        if (!isInitialized) {
            serial.writeLine("Erreur: système de déplacement non initialisé");
            return;
        }

        // Calculer la position cible
        const startX = odometry.getX();
        const startY = odometry.getY();
        const angle = odometry.getOrientationRad();
        const targetX = startX + Math.cos(angle) * distance;
        const targetY = startY + Math.sin(angle) * distance;

        // Réinitialiser les erreurs PID
        previousError = 0;
        integralError = 0;

        // Démarrer les moteurs en marche avant
        applyMotorCommand(baseSpeed, baseSpeed, Direction.Forward, Direction.Forward);

        // Durée maximale pour éviter une boucle infinie
        const startTime = input.runningTime();

        // Boucle jusqu'à atteindre la position cible
        while (true) {
            // Mettre à jour l'odométrie
            const encoderData = readEncoders();
            odometry.updateFromTicks(encoderData.leftDelta, encoderData.rightDelta);

            // Calculer l'erreur de distance restante
            const distanceRemaining = odometry.distanceTo(targetX, targetY);

            // Calculer l'erreur d'orientation (pour rester en ligne droite)
            const currentAngle = odometry.getOrientationRad();
            const angleError = angleBetween(angle, currentAngle);

            // Vérifier si l'objectif est atteint
            if (distanceRemaining < 5) { // 5mm de précision
                break;
            }

            // Calculer la correction d'orientation par PID
            let proportional = angleError * Kp;
            integralError += angleError;
            let integral = integralError * Ki;
            let derivative = (angleError - previousError) * Kd;
            previousError = angleError;

            // Calculer la correction de la vitesse pour maintenir la trajectoire
            let correction = proportional + integral + derivative;

            // Appliquer la correction aux moteurs
            let leftSpeed = baseSpeed - correction;
            let rightSpeed = baseSpeed + correction;

            // Ajuster la vitesse globale en fonction de la distance restante
            let speedMultiplier = 1.0;
            if (distanceRemaining < 100) {
                speedMultiplier = distanceRemaining / 100;
                speedMultiplier = Math.max(speedMultiplier, 0.3); // Au moins 30% de la vitesse
            }

            leftSpeed *= speedMultiplier;
            rightSpeed *= speedMultiplier;

            // Actualiser les vitesses des moteurs
            applyMotorCommand(leftSpeed, rightSpeed, Direction.Forward, Direction.Forward);

            // Vérifier le timeout
            if (input.runningTime() - startTime > MOVEMENT_TIMEOUT) {
                serial.writeLine("Timeout de déplacement");
                break;
            }

            // Petit délai pour éviter une utilisation trop intensive du processeur
            basic.pause(10);
        }

        // Arrêter les moteurs
        stopMotors();
    }

    /**
     * Calcule la plus petite différence angulaire entre deux angles en radians
     * @param angle1 Premier angle en radians
     * @param angle2 Deuxième angle en radians
     * @returns Différence d'angle normalisée entre -π et π
     */
    function angleBetween(angle1: number, angle2: number): number {
        let diff = angle1 - angle2;

        // Normaliser entre -π et π
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        return diff;
    }
}