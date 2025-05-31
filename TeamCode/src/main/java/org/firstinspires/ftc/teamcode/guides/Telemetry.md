## No you *can't* just Console.WriteLine() it! Lee's guide to Telemetry 101
Telemetry is essentially how you can output stuff, and it is shown on the Driver Hub. (Both before and during an OpMode, depending on what you've done.)

## So, how'd you code it?

Telemetry uses the format of Caption: Value(s), which are specified in the parameters of 
``telemetry.addData()``

If you have a single value, you can use ``("``<caption>``", ``<value>``)``, where the value may be a variable for example:
``telemetry.addData("Status", "initialised");``  <- e.g.

If you have multiple values which you want to output in one lines, you can instead use``("``Caption``", "``Format``", ``<value1>``, ``,<value2>``)``
The format string follows the usual Java formatting rules (here's some examples: https://www.geeksforgeeks.org/formatted-output-in-java/) however most commonly you need to output rounded numbers, like such:
``"%7d :%7d"`` (this specifies that there are 2 numbers provided, both of which should be rounded to 7 decimal places)

Importantly, everytime you want to add something to telemetry, you have to add
``telemetry.update();``
afterwards. If you are adding multiple lines in a row, you only need to update it once after all of them.

## Important note, if you use variables in your telemetry, you do not need to continuously update it! 
## Once you have updated so the line is added, the output will automatically update to the content of the variable(s).


TYPES IMPORTANT f/d