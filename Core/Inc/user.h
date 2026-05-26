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
 * Runs the main user application loop.
 *
 * This function should be called repeatedly from the main while loop. It
 * handles periodic control updates when the control update flag is set.
 */
void user_loop(void);

#endif /* INC_USER_H_ */
