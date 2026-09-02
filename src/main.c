#include <stdint.h>
#include <stdbool.h>
#include <xil_printf.h>

// Memory-mapped MicroBlaze peripherals
#define BUTTONS     (* (unsigned volatile *) 0x40000000)
#define JA          (* (unsigned volatile *) 0x40001000)
#define JA_DDR      (* (unsigned volatile *) 0x40001004)
#define JB          (* (unsigned volatile *) 0x40002000)
#define JB_DDR      (* (unsigned volatile *) 0x40002004)
#define JC          (* (unsigned volatile *) 0x40003000)
#define JC_DDR      (* (unsigned volatile *) 0x40003004)
#define JXADC       (* (unsigned volatile *) 0x40004000)
#define JXADC_DDR   (* (unsigned volatile *) 0x40004004)
#define LEDS        (* (unsigned volatile *) 0x40005000)
#define ANODES      (* (unsigned volatile *) 0x40006000)
#define SEVEN_SEG   (* (unsigned volatile *) 0x40006008)
#define SWITCHES    (* (unsigned volatile *) 0x40007000)
#define UART        (* (unsigned volatile *) 0x40008000)

// Hardware timer register map
#define ITP (uint32_t *)
const uint32_t *TIMERS[] = {
    ITP 0x40009000, ITP 0x40009010,
    ITP 0x4000A000, ITP 0x4000A010,
    ITP 0x4000B000, ITP 0x4000B010,
    ITP 0x4000C000, ITP 0x4000C010
};
const volatile uint32_t TCSR_OFFSET = 0;
const volatile uint32_t TLR_OFFSET = 1;
const volatile uint32_t TCR_OFFSET = 2;

// Ultrasonic sensor pin offsets
#define TRIG_OFFSET 0
#define ECHO_OFFSET 1

// Button offsets
#define BTNU_OFFSET 3
#define BTNL_OFFSET 2
#define BTNR_OFFSET 1
#define BTND_OFFSET 0

// Motor-driver and quadrature-encoder offsets
#define L_PWM_OFFSET 0
#define LEFT1_OFFSET 1
#define LEFT2_OFFSET 2
#define R_PWM_OFFSET 3
#define RIGHT2_OFFSET 4
#define RIGHT1_OFFSET 5
#define L1_QUAD_ENC_OFFSET 0
#define R1_QUAD_ENC_OFFSET 1

#define DUTY_CYCLE_MAX 100
#define FIXED_DUTY 40
#define TICKS_PER_INCH 46.314
#define TICKS_PER_90_RIGHT 235
#define TICKS_PER_90_LEFT 230
#define TIMEOUT 100000

void configure_motors();
void turn_on_left_motor();
void turn_on_right_motor();
void turn_off_left_motor();
void turn_off_right_motor();
void move_motors_with_pwm(uint8_t duty_cycle);
uint32_t read_L1_quad_enc(_Bool reset);
uint32_t read_R1_quad_enc(_Bool reset);

_Bool btnU_is_pressed();
_Bool btnD_is_pressed();
_Bool btnL_is_pressed();
_Bool btnR_is_pressed();

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

void configure_timers();
void reset_timer(uint8_t timer_number);
uint32_t get_timer_value_us(uint8_t timer_number);

void configure_ultrasonic_sensor();
void set_trig_pin();
void clear_trig_pin();
_Bool read_echo_pin();
uint16_t read_ultrasonic_sensor();

void update_sseg(uint8_t segment_data[4]);

void stop_motors();
void set_reverse_direction();
void set_forward_direction();

void go_straight(uint32_t target, _Bool *moving_straight);
void drive_reverse(uint32_t target, _Bool *moving_reverse);
void turn_right();
void turn_left();
void spin();

static const uint8_t sseg_lut[16] = {
    0b11000000, 0b11111001, 0b10100100, 0b10110000,
    0b10011001, 0b10010010, 0b10000010, 0b11111000,
    0b10000000, 0b10011000, 0b10001000, 0b10000011,
    0b11000110, 0b10100001, 0b10000110, 0b10001110,
};

int main() {
    configure_timers();
    LEDS = 0x00;
    ANODES = 0xF;
    configure_motors();

    uint8_t digits[4];
    uint8_t segment_data[4];
    _Bool moving_straight = false;
    _Bool moving_reverse = false;

    enum { start, move_y, turn, move_x, end } state = start;

    while (1) {
        uint32_t y_coordinate = SWITCHES & 0x1F;
        uint32_t x_coordinate = (SWITCHES >> 6) & 0x1F;
        uint32_t y_target = y_coordinate * TICKS_PER_INCH;
        uint32_t x_target = x_coordinate * TICKS_PER_INCH;

        _Bool negy = (SWITCHES >> 5) & 0x1;
        _Bool negx = (SWITCHES >> 11) & 0x1;

        digits[3] = (x_coordinate / 10) % 10;
        digits[2] = x_coordinate % 10;
        digits[1] = (y_coordinate / 10) % 10;
        digits[0] = y_coordinate % 10;

        for (uint8_t i = 0; i < 4; i++) {
            segment_data[i] = sseg_lut[digits[i]];
        }

        if (negy) {
            segment_data[1] &= ~(1 << 7);
        } else {
            segment_data[1] |= (1 << 7);
        }

        if (negx) {
            segment_data[3] &= ~(1 << 7);
        } else {
            segment_data[3] |= (1 << 7);
        }

        update_sseg(segment_data);

        switch (state) {
            case start:
                if (btnD_is_pressed()) {
                    read_L1_quad_enc(true);
                    read_R1_quad_enc(true);

                    if (negy) {
                        moving_reverse = true;
                    } else {
                        moving_straight = true;
                    }
                    state = move_y;
                }
                break;

            case move_y:
                if (negy) {
                    drive_reverse(y_target, &moving_reverse);
                    if (!moving_reverse) {
                        state = turn;
                    }
                } else {
                    go_straight(y_target, &moving_straight);
                    if (!moving_straight) {
                        state = turn;
                    }
                }
                break;

            case turn:
                if (negx) {
                    delay_ms(1000);
                    turn_left();
                } else {
                    delay_ms(1000);
                    turn_right();
                }

                read_L1_quad_enc(true);
                read_R1_quad_enc(true);
                moving_straight = true;
                state = move_x;
                break;

            case move_x:
                go_straight(x_target, &moving_straight);
                if (!moving_straight) {
                    state = end;
                }
                break;

            case end:
                stop_motors();
                break;
        }
    }

    return 0;
}

void configure_ultrasonic_sensor() {
    JB_DDR &= ~(1 << TRIG_OFFSET);
    JB_DDR |= (1 << ECHO_OFFSET);
}

void set_trig_pin() {
    JB |= (1 << TRIG_OFFSET);
}

void clear_trig_pin() {
    JB &= ~(1 << TRIG_OFFSET);
}

_Bool read_echo_pin() {
    return ((JB >> ECHO_OFFSET) & 1);
}

uint16_t read_ultrasonic_sensor() {
    static uint16_t distance = 0;
    static enum {
        SEND_TRIG,
        WAIT_FOR_ECHO,
        COUNT_ECHO_DURATION,
        COOLDOWN
    } state;

    switch (state) {
        case SEND_TRIG:
            reset_timer(0);
            set_trig_pin();
            while (get_timer_value_us(0) < 10);
            clear_trig_pin();
            reset_timer(0);
            state = WAIT_FOR_ECHO;
            break;

        case WAIT_FOR_ECHO:
            if (read_echo_pin()) {
                reset_timer(0);
                state = COUNT_ECHO_DURATION;
            } else if (get_timer_value_us(0) > TIMEOUT) {
                state = COOLDOWN;
            }
            break;

        case COUNT_ECHO_DURATION:
            if (read_echo_pin() == 0) {
                uint32_t flight_time = get_timer_value_us(0);
                distance = flight_time / 148;
                state = COOLDOWN;
            } else if (get_timer_value_us(0) > TIMEOUT) {
                state = COOLDOWN;
            }
            break;

        case COOLDOWN:
            if (get_timer_value_us(7) >= 100000) {
                state = SEND_TRIG;
                reset_timer(7);
            }
            break;
    }

    return distance;
}

void update_sseg(uint8_t segment_data[4]) {
    static uint8_t anode_count;

    anode_count++;
    if (anode_count == 4) {
        anode_count = 0;
    }

    switch (anode_count) {
        case 0:
            SEVEN_SEG = segment_data[0];
            ANODES = 0b1110;
            break;
        case 1:
            SEVEN_SEG = segment_data[1];
            ANODES = 0b1101;
            break;
        case 2:
            SEVEN_SEG = segment_data[2];
            ANODES = 0b1011;
            break;
        case 3:
            SEVEN_SEG = segment_data[3];
            ANODES = 0b0111;
            break;
    }
}

void configure_timers() {
    for (int timer_number = 0; timer_number < 8; timer_number++) {
        uint32_t *timer_base_address = ITP(TIMERS[timer_number]);
        uint32_t *tlr = timer_base_address + TLR_OFFSET;
        uint32_t *tcsr = timer_base_address + TCSR_OFFSET;
        *(tlr) = 0x00000000;
        *(tcsr) = 0b010010010001;
    }
}

void reset_timer(uint8_t timer_number) {
    if (timer_number > 7) {
        return;
    }

    uint32_t *timer_base_address = ITP(TIMERS[timer_number]);
    volatile uint32_t *tcsr = timer_base_address + TCSR_OFFSET;

    *tcsr &= ~(1 << 7);
    *tcsr |= 1 << 5;
    *tcsr &= ~(1 << 5);
    *tcsr |= 1 << 7;
}

uint32_t get_timer_value_us(uint8_t timer_number) {
    if (timer_number > 7) {
        return 0;
    }

    uint32_t *timer_base_address = ITP(TIMERS[timer_number]);
    volatile uint32_t *tcr = timer_base_address + TCR_OFFSET;
    return (*tcr) / 100;
}

void configure_motors() {
    JC_DDR = 0x00;
    JA_DDR = 0xFF;

    JC |= (1 << LEFT1_OFFSET);
    JC &= ~(1 << LEFT2_OFFSET);

    JC |= (1 << RIGHT2_OFFSET);
    JC &= ~(1 << RIGHT1_OFFSET);
}

void turn_on_left_motor() {
    JC |= (1 << L_PWM_OFFSET);
}

void turn_off_left_motor() {
    JC &= ~(1 << L_PWM_OFFSET);
}

void turn_on_right_motor() {
    JC |= (1 << R_PWM_OFFSET);
}

void turn_off_right_motor() {
    JC &= ~(1 << R_PWM_OFFSET);
}

void move_motors_with_pwm(uint8_t duty_cycle) {
    static uint8_t pwm_counter = 0;

    pwm_counter++;
    if (pwm_counter == DUTY_CYCLE_MAX) {
        pwm_counter = 0;
    }

    if (pwm_counter < duty_cycle) {
        JC |= (1 << L_PWM_OFFSET);
        JC |= (1 << R_PWM_OFFSET);
    } else {
        JC &= ~(1 << L_PWM_OFFSET);
        JC &= ~(1 << R_PWM_OFFSET);
    }
}

uint32_t read_L1_quad_enc(_Bool reset) {
    static uint32_t cnt = 0;
    static _Bool quad_enc_last_state = 0;

    if (reset) {
        cnt = 0;
        quad_enc_last_state = 0;
    }

    if ((quad_enc_last_state == 0) && (JA & (1 << L1_QUAD_ENC_OFFSET))) {
        cnt++;
    }

    quad_enc_last_state = JA & (1 << L1_QUAD_ENC_OFFSET);
    return cnt;
}

uint32_t read_R1_quad_enc(_Bool reset) {
    static uint32_t cnt = 0;
    static _Bool quad_enc_last_state = 0;

    if (reset) {
        cnt = 0;
        quad_enc_last_state = 0;
    }

    if ((quad_enc_last_state == 0) && (JA & (1 << R1_QUAD_ENC_OFFSET))) {
        cnt++;
    }

    quad_enc_last_state = JA & (1 << R1_QUAD_ENC_OFFSET);
    return cnt;
}

void delay_ms(uint32_t ms) {
    const uint32_t ONE_MS_TOP = 7700;
    for (volatile uint32_t t = 0; t < ms; t++) {
        for (volatile uint32_t count = 0; count < ONE_MS_TOP; count++);
    }
}

void delay_us(uint32_t us) {
    const uint32_t ONE_US_TOP = 8;
    for (volatile uint32_t t = 0; t < us; t++) {
        for (volatile uint32_t count = 0; count < ONE_US_TOP; count++);
    }
}

_Bool btnU_is_pressed() {
    static _Bool current_value = false, previous_value = false;
    previous_value = current_value;
    current_value = BUTTONS & (1 << BTNU_OFFSET);
    return (current_value && !previous_value);
}

_Bool btnD_is_pressed() {
    static _Bool current_value = false, previous_value = false;
    previous_value = current_value;
    current_value = BUTTONS & (1 << BTND_OFFSET);
    return (current_value && !previous_value);
}

_Bool btnL_is_pressed() {
    static _Bool current_value = false, previous_value = false;
    previous_value = current_value;
    current_value = BUTTONS & (1 << BTNL_OFFSET);
    return (current_value && !previous_value);
}

_Bool btnR_is_pressed() {
    static _Bool current_value = false, previous_value = false;
    previous_value = current_value;
    current_value = BUTTONS & (1 << BTNR_OFFSET);
    return (current_value && !previous_value);
}

void stop_motors() {
    turn_off_left_motor();
    turn_off_right_motor();
}

void set_reverse_direction() {
    JC &= ~(1 << LEFT1_OFFSET);
    JC |= (1 << LEFT2_OFFSET);

    JC &= ~(1 << RIGHT2_OFFSET);
    JC |= (1 << RIGHT1_OFFSET);
}

void set_forward_direction() {
    JC |= (1 << LEFT1_OFFSET);
    JC &= ~(1 << LEFT2_OFFSET);

    JC |= (1 << RIGHT2_OFFSET);
    JC &= ~(1 << RIGHT1_OFFSET);
}

void go_straight(uint32_t target, _Bool *moving_straight) {
    uint32_t left_quad_enc_value = read_L1_quad_enc(false);
    uint32_t right_quad_enc_value = read_R1_quad_enc(false);

    if (*moving_straight) {
        set_forward_direction();

        if (left_quad_enc_value <= target || right_quad_enc_value <= target) {
            if (left_quad_enc_value > right_quad_enc_value) {
                JC &= ~(1 << L_PWM_OFFSET);
            } else {
                JC |= (1 << L_PWM_OFFSET);
            }

            if (right_quad_enc_value > left_quad_enc_value) {
                JC &= ~(1 << R_PWM_OFFSET);
            } else {
                JC |= (1 << R_PWM_OFFSET);
            }
        } else {
            turn_off_left_motor();
            turn_off_right_motor();
            *moving_straight = false;
        }
    }
}

void drive_reverse(uint32_t target, _Bool *moving_reverse) {
    set_reverse_direction();

    if (*moving_reverse == true) {
        uint32_t left = read_L1_quad_enc(false);
        uint32_t right = read_R1_quad_enc(false);

        if (left >= target || right >= target) {
            turn_off_left_motor();
            turn_off_right_motor();
            *moving_reverse = false;
            return;
        }

        if (left > right) {
            JC &= ~(1 << L_PWM_OFFSET);
        } else {
            JC |= (1 << L_PWM_OFFSET);
        }

        if (right > left) {
            JC &= ~(1 << R_PWM_OFFSET);
        } else {
            JC |= (1 << R_PWM_OFFSET);
        }
    }
}

void turn_right() {
    JC |= (1 << LEFT1_OFFSET);
    JC &= ~(1 << LEFT2_OFFSET);

    JC &= ~(1 << RIGHT2_OFFSET);
    JC |= (1 << RIGHT1_OFFSET);

    read_L1_quad_enc(true);
    read_R1_quad_enc(true);

    while (1) {
        uint32_t left = read_L1_quad_enc(false);
        uint32_t right = read_R1_quad_enc(false);

        if (left >= TICKS_PER_90_RIGHT || right >= TICKS_PER_90_RIGHT) {
            break;
        }

        JC |= (1 << L_PWM_OFFSET);
        JC |= (1 << R_PWM_OFFSET);

        if (left > right) {
            JC &= ~(1 << L_PWM_OFFSET);
        } else if (right > left) {
            JC &= ~(1 << R_PWM_OFFSET);
        }
    }

    stop_motors();
}

void turn_left() {
    JC &= ~(1 << LEFT1_OFFSET);
    JC |= (1 << LEFT2_OFFSET);

    JC |= (1 << RIGHT2_OFFSET);
    JC &= ~(1 << RIGHT1_OFFSET);

    read_L1_quad_enc(true);
    read_R1_quad_enc(true);

    while (1) {
        uint32_t left = read_L1_quad_enc(false);
        uint32_t right = read_R1_quad_enc(false);

        if (left >= TICKS_PER_90_LEFT || right >= TICKS_PER_90_LEFT) {
            break;
        }

        JC |= (1 << L_PWM_OFFSET);
        JC |= (1 << R_PWM_OFFSET);

        if (left > right) {
            JC &= ~(1 << L_PWM_OFFSET);
        } else if (right > left) {
            JC &= ~(1 << R_PWM_OFFSET);
        }
    }

    stop_motors();
}

void spin() {
    configure_motors();

    JC |= (1 << LEFT1_OFFSET);
    JC &= ~(1 << LEFT2_OFFSET);

    JC &= ~(1 << RIGHT2_OFFSET);
    JC |= (1 << RIGHT1_OFFSET);

    read_L1_quad_enc(true);
    read_R1_quad_enc(true);

    while (1) {
        uint32_t left = read_L1_quad_enc(false);
        uint32_t right = read_R1_quad_enc(false);

        if (left >= (TICKS_PER_90_LEFT * 4) &&
            right >= (TICKS_PER_90_LEFT * 4)) {
            break;
        }

        JC |= (1 << L_PWM_OFFSET);
        JC |= (1 << R_PWM_OFFSET);

        if (right > left) {
            JC &= ~(1 << R_PWM_OFFSET);
        } else {
            JC |= (1 << R_PWM_OFFSET);
        }
    }

    stop_motors();
}
