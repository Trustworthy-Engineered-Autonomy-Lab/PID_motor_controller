#ifndef INC_USER_H_
#define INC_USER_H_

/*
 * Initializes the user application layer.
 *
 * This function is called once during system startup. It initializes the
 * application modules and starts the timer/PWM functions required by the
 * motor-control workflow.
 */
void user_init(void);

/*
 * Runs one control-loop update.
 *
 * This function is called directly from the control timer callback. It
 * should not be called repeatedly from the main while loop.
 */
void user_loop(void);

#endif /* INC_USER_H_ */
