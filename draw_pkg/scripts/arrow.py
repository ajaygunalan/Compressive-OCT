import turtle

# Create a turtle instance
arrow = turtle.Turtle()

# Set the speed of the turtle (0 is the fastest)
arrow.speed(1)

# Set the color of the arrow
arrow.color("blue")

# Draw the arrow
arrow.left(135)  # Turn left at a specific angle
arrow.forward(100)  # Move forward
arrow.left(90)  # Turn right at a specific angle
arrow.forward(100)  # Move forward


# Keep the turtle window open
turtle.done()
