import com.cra.figaro.algorithm.factored.VariableElimination
import com.cra.figaro.language._
import com.cra.figaro.library.atomic.discrete.FromRange

object TwoDice {
  val dice_1 = FromRange(1,7)
  val dice_2 = FromRange(1,7)

  def sum_prob() {
    val sum = Apply(dice_1, dice_2,
      (d1: Int, d2: Int) => d1 + d2)
    println("2: " + VariableElimination.probability(sum, 2))
    println("3: " + VariableElimination.probability(sum, 3))
    println("4: " + VariableElimination.probability(sum, 4))
    println("5: " + VariableElimination.probability(sum, 5))
    println("6: " + VariableElimination.probability(sum, 6))
    println("7: " + VariableElimination.probability(sum, 7))
    println("8: " + VariableElimination.probability(sum, 8))
    println("9: " + VariableElimination.probability(sum, 9))
    println("10: " + VariableElimination.probability(sum, 10))
    println("11: " + VariableElimination.probability(sum, 11))
    println("12: " + VariableElimination.probability(sum, 12))
  }

  def sum_prob_with_six() {
    val sum_gt_8 = Apply(dice_1, dice_2, 
      (d1: Int, d2: Int) => d1 == 6 && d1 + d2 > 8)
    println(VariableElimination.probability(sum_gt_8, true))
  }

  def jail() {
    val pair_six = Apply(dice_1, dice_2, 
      (d1: Int, d2: Int) => d1 == 6 && d2 == 6)

    println(scala.math.pow(VariableElimination.probability(pair_six, true), 3))
  }

  def main(args : Array[String]) = {
    sum_prob()
    sum_prob_with_six()
    jail()
  }
}

