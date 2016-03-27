import com.cra.figaro.algorithm.factored.VariableElimination
import com.cra.figaro.language._
import com.cra.figaro.library.atomic.discrete.FromRange

object Game {
  val dice_4 = FromRange(1, 5)
  val dice_6 = FromRange(1, 7)
  val dice_8 = FromRange(1, 9)
  val dice_12 = FromRange(1, 13)
  val dice_20 = FromRange(1, 21)

  val spinner = FromRange(1, 6)

  def spin() = {
    val dice = Chain(spinner, (d: Int) =>
      d match {
        case 1 => dice_4
	case 2 => dice_6
	case 3 => dice_8
	case 4 => dice_12
	case 5 => dice_20
      })
    
    println("P(dice=12) = " + VariableElimination.probability(spinner, 4))
    println("P(roll=7) = " + VariableElimination.probability(dice, 7))
    
    dice.observe(7)
    println("P(dice=12|roll=7) = " + VariableElimination.probability(spinner, 4))
    dice.unobserve()

    spinner.observe(4)
    println("P(roll=7|dice=12) = " + VariableElimination.probability(dice, 7))
    spinner.unobserve()
  }

  def main(args : Array[String]) = {
    spin() 
  }
}

