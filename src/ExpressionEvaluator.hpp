#pragma once
#include <cmath>
#include <functional>
#include <map>
#include <simd/simd.h>
#include <string>
#include <vector>

class ExpressionEvaluator {
public:
  struct Context {
    std::map<std::string, float> parameters;
    std::map<std::string, float> materialProperties;
    std::map<std::string, simd::float3> namedPoints;
    std::map<std::string, float> domainVariables;
    std::function<float(const std::string &field, simd::float3 point)>
        fieldEvaluator;
  };

  static float resolve(const std::string &value, const Context &ctx);
  static bool isExpression(const std::string &value);
  static float evaluate(const std::string &expr, const Context &ctx);

private:
  enum TokenType {
    TOK_NUMBER, TOK_IDENTIFIER, TOK_PLUS, TOK_MINUS, TOK_STAR, TOK_SLASH,
    TOK_CARET, TOK_LPAREN, TOK_RPAREN, TOK_LBRACKET, TOK_RBRACKET,
    TOK_COMMA, TOK_COLON, TOK_DOT, TOK_AT, TOK_EOF
  };

  struct Token {
    TokenType type;
    std::string text;
    float numValue;
  };

  static std::vector<Token> tokenize(const std::string &expr);

  struct Parser {
    const std::vector<Token> &tokens;
    const Context &ctx;
    size_t pos;

    Parser(const std::vector<Token> &t, const Context &c)
        : tokens(t), ctx(c), pos(0) {}

    const Token &current() const;
    const Token &advance();
    bool match(TokenType type);
    void expect(TokenType type);

    float parseExpr();
    float parseTerm();
    float parsePower();
    float parseUnary();
    float parseAtom();
    float parseReference();
    float parseFunctionCall(const std::string &funcName);
    simd::float3 parsePointConstruction();
  };
};
