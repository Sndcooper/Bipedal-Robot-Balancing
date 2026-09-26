with open('src/main.cpp', 'r', encoding='utf-8') as f:
    text = f.read()

motor_defs = '''
// -- DC MOTOR DEFINITIONS --
#define ENA PA0  // Speed (PWM) Left
#define IN1 PB12 // Direction 1 Left
#define IN2 PB13 // Direction 2 Left

#define ENB PA1  // Speed (PWM) Right
#define IN3 PB14 // Direction 1 Right
#define IN4 PB15 // Direction 2 Right

void setMotors(int leftSpeed, int rightSpeed);
'''

if 'DC MOTOR DEFINITIONS' not in text:
    text = text.replace('// -- Modes --', motor_defs + '\n// -- Modes --')

setup_motors = '''
    // Setup Motor Pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    setMotors(0, 0); // Stop both
'''

if 'pinMode(ENA' not in text:
    text = text.replace('Serial1.begin(115200);', 'Serial1.begin(115200);\n' + setup_motors)

motor_cmd = '''    else if (type == "MOT") {
        int comma2 = rest.indexOf(',');
        int leftSpeed = rest.substring(0, comma2).toInt();
        int rightSpeed = rest.substring(comma2 + 1).toInt();
        setMotors(leftSpeed, rightSpeed);
        Serial1.print("MOT -> Left: "); Serial1.print(leftSpeed);
        Serial1.print(" | Right: "); Serial1.println(rightSpeed);
    }'''

search_str = '    else {\n        Serial1.println("ERR: Unknown command type: " + type);\n    }'
if 'type == "MOT"' not in text:
    text = text.replace(search_str, motor_cmd + '\n' + search_str)

func_motors = '''
// ----------------------------------------
//  DC MOTOR CONTROL
// ----------------------------------------
void setMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -leftSpeed);
    }
    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -rightSpeed);
    }
}
'''

if 'void setMotors' not in text:
    text += '\n' + func_motors

with open('src/main.cpp', 'w', encoding='utf-8') as f:
    f.write(text)
