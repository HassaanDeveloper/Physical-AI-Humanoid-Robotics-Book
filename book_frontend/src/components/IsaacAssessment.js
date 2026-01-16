import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './IsaacAssessment.module.css';

const IsaacAssessment = ({ title, questions, showAnswers = true }) => {
  const [selectedAnswers, setSelectedAnswers] = useState({});
  const [showResults, setShowResults] = useState(false);

  const handleAnswerSelect = (questionIndex, answerIndex) => {
    setSelectedAnswers({
      ...selectedAnswers,
      [questionIndex]: answerIndex
    });
  };

  const handleSubmit = () => {
    setShowResults(true);
  };

  const handleReset = () => {
    setSelectedAnswers({});
    setShowResults(false);
  };

  const getAnswerClass = (questionIndex, answerIndex, isCorrect) => {
    if (!showResults) return '';

    if (selectedAnswers[questionIndex] === answerIndex) {
      return isCorrect ? styles.correctAnswer : styles.incorrectAnswer;
    }

    if (isCorrect && showResults) {
      return styles.correctAnswer;
    }

    return '';
  };

  return (
    <div className={styles.assessmentContainer}>
      <div className={styles.assessmentHeader}>
        <h3 className={styles.assessmentTitle}>
          <span className={styles.isaacPlatformIcon}>A</span>
          {title}
        </h3>
      </div>

      <div className={styles.questionsContainer}>
        {questions.map((question, qIndex) => (
          <div key={qIndex} className={styles.questionContainer}>
            <div className={styles.questionText}>
              <strong>{qIndex + 1}. {question.question}</strong>
            </div>

            <div className={styles.answersContainer}>
              {question.answers.map((answer, aIndex) => (
                <div
                  key={aIndex}
                  className={clsx(styles.answerOption, getAnswerClass(qIndex, aIndex, answer.isCorrect), {
                    [styles.selectedAnswer]: selectedAnswers[qIndex] === aIndex
                  })}
                  onClick={() => !showResults && handleAnswerSelect(qIndex, aIndex)}
                >
                  <span className={styles.answerLetter}>{String.fromCharCode(97 + aIndex)}. </span>
                  <span className={styles.answerText}>{answer.text}</span>

                  {showResults && answer.isCorrect && (
                    <span className={styles.answerIndicator}>✓</span>
                  )}

                  {showResults && selectedAnswers[qIndex] === aIndex && !answer.isCorrect && (
                    <span className={styles.answerIndicator}>✗</span>
                  )}
                </div>
              ))}
            </div>

            {showAnswers && question.explanation && showResults && (
              <div className={styles.explanation}>
                <strong>Explanation:</strong> {question.explanation}
              </div>
            )}
          </div>
        ))}
      </div>

      <div className={styles.controlsContainer}>
        {!showResults ? (
          <button className={styles.submitButton} onClick={handleSubmit}>
            Submit Answers
          </button>
        ) : (
          <div className={styles.resultsContainer}>
            <div className={styles.resultsSummary}>
              Correct Answers: {questions.filter((_, index) => {
                const userAnswer = selectedAnswers[index];
                return userAnswer !== undefined &&
                       questions[index].answers[userAnswer]?.isCorrect;
              }).length} / {questions.length}
            </div>
            <button className={styles.resetButton} onClick={handleReset}>
              Reset Assessment
            </button>
          </div>
        )}
      </div>
    </div>
  );
};

export default IsaacAssessment;