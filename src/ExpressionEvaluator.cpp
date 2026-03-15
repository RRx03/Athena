#include "ExpressionEvaluator.hpp"
#include <cctype>
#include <cmath>
#include <stdexcept>

float ExpressionEvaluator::resolve(const std::string &value,
                                   const Context &ctx) {
  try {
    size_t pos;
    float num = std::stof(value, &pos);
    if (pos == value.size())
      return num;
  } catch (...) {}
  return evaluate(value, ctx);
}

bool ExpressionEvaluator::isExpression(const std::string &value) {
  if (value.empty()) return false;
  return value.find("param:") == 0 || value.find("material:") == 0 ||
         value.find("point:") == 0 || value.find("field:") == 0 ||
         value.find("domain:") == 0 || value.find("expr:") == 0;
}

float ExpressionEvaluator::evaluate(const std::string &expr,
                                    const Context &ctx) {
  std::string toParse = expr;
  if (toParse.find("expr:") == 0)
    toParse = toParse.substr(5);

  auto tokens = tokenize(toParse);
  Parser parser(tokens, ctx);
  float result = parser.parseExpr();

  if (parser.pos < tokens.size() && tokens[parser.pos].type != TOK_EOF)
    throw std::runtime_error("ExpressionEvaluator: tokens restants dans '" + expr + "'");

  return result;
}

// ═══════════════ Tokenizer ═══════════════

std::vector<ExpressionEvaluator::Token>
ExpressionEvaluator::tokenize(const std::string &expr) {
  std::vector<Token> tokens;
  size_t i = 0;

  while (i < expr.size()) {
    char c = expr[i];

    if (std::isspace(c)) { i++; continue; }

    if (std::isdigit(c) || (c == '.' && i + 1 < expr.size() && std::isdigit(expr[i + 1]))) {
      size_t start = i;
      while (i < expr.size() &&
             (std::isdigit(expr[i]) || expr[i] == '.' || expr[i] == 'e' ||
              expr[i] == 'E' ||
              ((expr[i] == '+' || expr[i] == '-') && i > start &&
               (expr[i - 1] == 'e' || expr[i - 1] == 'E'))))
        i++;
      std::string numStr = expr.substr(start, i - start);
      tokens.push_back({TOK_NUMBER, numStr, std::stof(numStr)});
      continue;
    }

    if (std::isalpha(c) || c == '_') {
      size_t start = i;
      while (i < expr.size() && (std::isalnum(expr[i]) || expr[i] == '_'))
        i++;
      tokens.push_back({TOK_IDENTIFIER, expr.substr(start, i - start), 0});
      continue;
    }

    Token tok;
    tok.numValue = 0;
    tok.text = std::string(1, c);
    switch (c) {
    case '+': tok.type = TOK_PLUS; break;
    case '-': tok.type = TOK_MINUS; break;
    case '*': tok.type = TOK_STAR; break;
    case '/': tok.type = TOK_SLASH; break;
    case '^': tok.type = TOK_CARET; break;
    case '(': tok.type = TOK_LPAREN; break;
    case ')': tok.type = TOK_RPAREN; break;
    case '[': tok.type = TOK_LBRACKET; break;
    case ']': tok.type = TOK_RBRACKET; break;
    case ',': tok.type = TOK_COMMA; break;
    case ':': tok.type = TOK_COLON; break;
    case '.': tok.type = TOK_DOT; break;
    case '@': tok.type = TOK_AT; break;
    default:
      throw std::runtime_error(std::string("ExpressionEvaluator: caractère inattendu '") + c + "'");
    }
    tokens.push_back(tok);
    i++;
  }

  tokens.push_back({TOK_EOF, "", 0});
  return tokens;
}

// ═══════════════ Parser ═══════════════

const ExpressionEvaluator::Token &ExpressionEvaluator::Parser::current() const {
  return tokens[pos];
}
const ExpressionEvaluator::Token &ExpressionEvaluator::Parser::advance() {
  return tokens[pos++];
}
bool ExpressionEvaluator::Parser::match(TokenType type) {
  if (current().type == type) { pos++; return true; }
  return false;
}
void ExpressionEvaluator::Parser::expect(TokenType type) {
  if (!match(type))
    throw std::runtime_error("ExpressionEvaluator: attendu type " +
                             std::to_string(type) + " trouvé '" + current().text + "'");
}

float ExpressionEvaluator::Parser::parseExpr() {
  float result = parseTerm();
  while (current().type == TOK_PLUS || current().type == TOK_MINUS) {
    if (match(TOK_PLUS)) result += parseTerm();
    else { match(TOK_MINUS); result -= parseTerm(); }
  }
  return result;
}

float ExpressionEvaluator::Parser::parseTerm() {
  float result = parsePower();
  while (current().type == TOK_STAR || current().type == TOK_SLASH) {
    if (match(TOK_STAR)) result *= parsePower();
    else {
      match(TOK_SLASH);
      float d = parsePower();
      if (std::abs(d) < 1e-30f) throw std::runtime_error("Division par zéro");
      result /= d;
    }
  }
  return result;
}

float ExpressionEvaluator::Parser::parsePower() {
  float base = parseUnary();
  if (match(TOK_CARET)) {
    float exp = parseUnary();
    return std::pow(base, exp);
  }
  return base;
}

float ExpressionEvaluator::Parser::parseUnary() {
  if (match(TOK_MINUS)) return -parseUnary();
  return parseAtom();
}

float ExpressionEvaluator::Parser::parseAtom() {
  if (current().type == TOK_NUMBER) {
    float val = current().numValue;
    advance();
    return val;
  }

  if (match(TOK_LPAREN)) {
    float val = parseExpr();
    expect(TOK_RPAREN);
    return val;
  }

  if (current().type == TOK_IDENTIFIER) {
    std::string id = current().text;
    advance();

    if (current().type == TOK_COLON) {
      pos--;
      return parseReference();
    }

    if (current().type == TOK_LPAREN)
      return parseFunctionCall(id);

    auto dit = ctx.domainVariables.find(id);
    if (dit != ctx.domainVariables.end()) return dit->second;
    auto pit = ctx.parameters.find(id);
    if (pit != ctx.parameters.end()) return pit->second;

    throw std::runtime_error("ExpressionEvaluator: identifiant inconnu '" + id + "'");
  }

  throw std::runtime_error("ExpressionEvaluator: token inattendu '" + current().text + "'");
}

float ExpressionEvaluator::Parser::parseReference() {
  std::string prefix = current().text;
  advance();
  expect(TOK_COLON);

  if (prefix == "param") {
    std::string name = current().text; advance();
    auto it = ctx.parameters.find(name);
    if (it == ctx.parameters.end())
      throw std::runtime_error("Paramètre inconnu: " + name);
    return it->second;
  }

  if (prefix == "material") {
    std::string mat = current().text; advance();
    expect(TOK_DOT);
    std::string prop = current().text; advance();
    std::string key = mat + "." + prop;
    auto it = ctx.materialProperties.find(key);
    if (it == ctx.materialProperties.end())
      throw std::runtime_error("Propriété matériau inconnue: " + key);
    return it->second;
  }

  if (prefix == "point") {
    std::string name = current().text; advance();
    expect(TOK_DOT);
    std::string comp = current().text; advance();
    auto it = ctx.namedPoints.find(name);
    if (it == ctx.namedPoints.end())
      throw std::runtime_error("Point nommé inconnu: " + name);
    simd::float3 p = it->second;
    if (comp == "x" || comp == "r") return p.x;
    if (comp == "y" || comp == "theta") return p.y;
    if (comp == "z" || comp == "phi") return p.z;
    throw std::runtime_error("Composante invalide: " + comp);
  }

  if (prefix == "domain") {
    std::string name = current().text; advance();
    auto it = ctx.domainVariables.find(name);
    if (it == ctx.domainVariables.end())
      throw std::runtime_error("Variable domaine inconnue: " + name);
    return it->second;
  }

  if (prefix == "field") {
    std::string quantity = current().text; advance();
    expect(TOK_AT);

    simd::float3 point;
    if (current().type == TOK_LBRACKET) {
      point = parsePointConstruction();
    } else {
      std::string pointName = current().text; advance();
      auto it = ctx.namedPoints.find(pointName);
      if (it == ctx.namedPoints.end())
        throw std::runtime_error("Point inconnu pour field: " + pointName);
      point = it->second;
    }

    if (!ctx.fieldEvaluator)
      throw std::runtime_error("Pas de fieldEvaluator pour field:" + quantity);
    return ctx.fieldEvaluator(quantity, point);
  }

  throw std::runtime_error("Préfixe inconnu: " + prefix);
}

simd::float3 ExpressionEvaluator::Parser::parsePointConstruction() {
  expect(TOK_LBRACKET);
  float c0 = parseExpr();
  expect(TOK_COMMA);
  float c1 = parseExpr();

  if (match(TOK_COMMA)) {
    // 3 composantes : @[x, y, z] ou @[r, θ, z] → direct mapping
    float c2 = parseExpr();
    expect(TOK_RBRACKET);
    return simd::float3{c0, c1, c2};
  }

  // 2 composantes : @[r, z] en axisymétrique → float3(r, 0, z)
  // r va en .x (radial), z va en .z (axial), y=0 (θ=0 implicite)
  expect(TOK_RBRACKET);
  return simd::float3{c0, 0.0f, c1};
}

float ExpressionEvaluator::Parser::parseFunctionCall(const std::string &funcName) {
  expect(TOK_LPAREN);
  std::vector<float> args;
  if (current().type != TOK_RPAREN) {
    args.push_back(parseExpr());
    while (match(TOK_COMMA))
      args.push_back(parseExpr());
  }
  expect(TOK_RPAREN);

  if (funcName == "sin" && args.size() == 1) return std::sin(args[0]);
  if (funcName == "cos" && args.size() == 1) return std::cos(args[0]);
  if (funcName == "tan" && args.size() == 1) return std::tan(args[0]);
  if (funcName == "sqrt" && args.size() == 1) return std::sqrt(args[0]);
  if (funcName == "abs" && args.size() == 1) return std::abs(args[0]);
  if (funcName == "exp" && args.size() == 1) return std::exp(args[0]);
  if (funcName == "log" && args.size() == 1) return std::log(args[0]);
  if (funcName == "min" && args.size() == 2) return std::min(args[0], args[1]);
  if (funcName == "max" && args.size() == 2) return std::max(args[0], args[1]);
  if (funcName == "pow" && args.size() == 2) return std::pow(args[0], args[1]);
  if (funcName == "asin" && args.size() == 1) return std::asin(args[0]);
  if (funcName == "acos" && args.size() == 1) return std::acos(args[0]);
  if (funcName == "atan" && args.size() == 1) return std::atan(args[0]);
  if (funcName == "atan2" && args.size() == 2) return std::atan2(args[0], args[1]);
  if (funcName == "pi" && args.empty()) return M_PI;

  throw std::runtime_error("Fonction inconnue: " + funcName + "/" + std::to_string(args.size()));
}